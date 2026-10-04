"""Robot-facing stationary planning, with one recording and owned scene lifetime."""
from contextlib import ExitStack
from functools import partial
from time import perf_counter
import math
import numpy as np

from .base_planning import find_stationary_base
from .collision import PybulletServer
from .execution import high_qos, check_cancel
from .geometry import as_plane
from .planning import normalize_joint_ranges, _in_ranges
from .recording import recorded, current_run, loaded_versions, event
from .robot_adapter import kinematics_from_robot, resolve_arm_joint_names, _active_tool, configuration_from_values
from .robot_planning import json_input
from .stationary_region import StationaryRegion

STATIONARY_WORKFLOW_VERSION = 3


def _transition_limits(n, start, max_step, intervals, speed):
    step = np.broadcast_to(np.asarray(max_step, dtype=float), (6,)).copy()
    if not np.isfinite(step).all() or np.any(step <= 0):
        raise ValueError('max_joint_step must contain positive finite radians')
    times = None if intervals is None else np.asarray(intervals, dtype=float)
    expected = n-int(start is None)
    if times is not None and (times.shape != (expected,) or not np.isfinite(times).all() or np.any(times <= 0)):
        raise ValueError('time_intervals must contain {} finite positive durations'.format(expected))
    limits = None
    if speed is not None:
        if times is None:
            raise ValueError('max_joint_speed requires time_intervals')
        speed = np.broadcast_to(np.asarray(speed, dtype=float), (6,)).copy()
        if not np.isfinite(speed).all() or np.any(speed <= 0):
            raise ValueError('max_joint_speed must contain positive finite radians/sec')
        limits = np.tile(step, (n, 1))
        limits[int(start is None):] = np.minimum(step, times[:, None]*speed)
    return step, times, speed, limits


def _status(found, count, build_path, collision_check, speed_checked):
    if found.base_planes:
        text = 'Found base. All {} targets reachable; {} positions tested; wall distance {:.2f} m.'.format(
            count, found.validation_attempts, found.standoff)
        if not build_path:
            text += ' Joint path not requested (build_path=False); planned_tcp is empty.'
        elif not found.configurations:
            detail = found.disconnected_detail
            text += ' Joint path blocked by max_joint_step{}.'.format(' or max_joint_speed' if speed_checked else '')
            if detail:
                source = 'start' if detail['from_target'] < 0 else 'target {}'.format(detail['from_target']+1)
                text += ' Disconnected from {} to target {}.'.format(source, detail['to_target']+1)
        else:
            text += ' Complete joint path; {}.'.format(
                'configuration collisions checked, transitions unchecked' if collision_check else 'collision checks off')
        if not collision_check and not found.configurations:
            text += ' Collision checks off.'
        return text
    text = 'No valid base. {} positions tested.'.format(found.validation_attempts)
    failed = [d['failed_target_details'] for d in found.diagnostics if d.get('failed_target_details')]
    if failed:
        indices = sorted({d['target_index']+1 for d in failed})
        text += ' Failed at target{} {} of {}.'.format('s' if len(indices)>1 else '', ', '.join(map(str, indices)), count)
        from collections import Counter
        blockers = Counter()
        for detail in failed:
            blockers.update(detail['rejection_reasons'])
        text += (' Main collisions: '+ '; '.join(reason.replace('robot_arm_', '').replace('robot_', '')
                 for reason, _ in blockers.most_common(2))+'.') if blockers else ' IK or joint limits rejected the configurations.'
    elif found.initial_state_failure:
        text += ' Starting configuration collision: '+found.initial_state_failure+'.'
    else:
        text += ' Placement constraints or base collisions rejected the candidates.'
    return text+' See diagnostics for details.'


@recorded
@high_qos
def plan_stationary_base(robot, target_planes, *, candidate_planes=(), units_to_metres=1.,
                         current_pose=None, arm_in_base=None, arm_joint_names=None,
                         fixed_joint_values=None, group=None, parameters=None,
                         grid_spacing=None, base_height=0., yaw_steps=4,
                         rotation_steps=16, max_validation_attempts=3, max_joint_step=2.5,
                         joint_ranges=None, build_path=True, count_paths=False, fast_validation=True,
                         collision_check=True, collision_meshes=(), collision_options=None, scene=None,
                         time_intervals=None, max_joint_speed=None, cancel_check=None, progress=None,
                         search_strategy='heuristic', search_options=None):
    """Planes/meshes/grid_spacing use model units; JSON search/collision lengths use metres.

    External scenes stay open and must match robot/tool/environment. Only a
    clearance query option can accompany a supplied scene. Fixed joints are set
    explicitly every run. Cancellation raises PlanningCancelled; no partial
    trajectory is returned. valid means a complete configuration-collision-checked
    path, not swept/transition validation. Recording honors TOOLBOX_RECORDING.
    """
    started = perf_counter()
    run = current_run()
    def report(stage, **data):
        check_cancel(cancel_check)
        if 'elapsed_seconds' in data:
            data['search_elapsed_seconds'] = data.pop('elapsed_seconds')
        message = dict(stage=stage, elapsed_seconds=perf_counter()-started,
                       research_run=str(run.path) if run else None, **data)
        event('stationary.progress', **message)
        if progress:
            progress(message)
        check_cancel(cancel_check)
    report('setup')
    if search_strategy not in ('heuristic', 'adaptive'):
        raise ValueError('search_strategy must be heuristic or adaptive')
    adaptive = search_strategy == 'adaptive' and fast_validation and not count_paths
    search_settings = dict(json_input(search_options, {}))
    allowed_search = {'grid_size', 'yaw_steps', 'probe_count', 'probe_rotations', 'beam_width', 'refinement_steps', 'validation_probe_count', 'connected_finalists',
                      'max_full_checks', 'initial_reach', 'exploration_reach', 'tool_clearance',
                      'clearance_weight', 'heading_bias'}
    if set(search_settings)-allowed_search:
        raise ValueError('Unknown adaptive search options: '+str(sorted(set(search_settings)-allowed_search)))
    if adaptive:
        from inspect import signature
        from .adaptive_stationary import find_adaptive_stationary_base
        defaults={name:parameter.default for name,parameter in
                  signature(find_adaptive_stationary_base).parameters.items() if name in allowed_search}
        search_settings={**defaults,**search_settings}
    search_result = None
    if robot is None:
        raise ValueError('Connect robot: its arm geometry, tool and collision model are required')
    if not math.isfinite(units_to_metres) or units_to_metres <= 0:
        raise ValueError('units_to_metres must be positive; use 0.001 for millimetre inputs')
    if _active_tool(robot, group) is None:
        raise ValueError('Attach the calibrated active tool before stationary validation')
    targets = [as_plane(t, units_to_metres) for t in target_planes]
    if not targets:
        raise ValueError('At least one target required for placement search')
    names = resolve_arm_joint_names(robot, arm_joint_names, group=group)
    joints = {j.name: j for j in robot.model.get_configurable_joints()}
    if len(names) != 6 or len(set(names)) != 6 or any(n not in joints or joints[n].type not in (0, 1) for n in names):
        raise ValueError('Provide six distinct UR revolute arm_joint_names in analytic order')
    fixed = dict(json_input(fixed_joint_values, {}))
    if hasattr(current_pose, 'joint_values'):
        if len(current_pose.joint_names) != len(current_pose.joint_values) or len(set(current_pose.joint_names)) != len(current_pose.joint_names):
            raise ValueError('Starting Configuration requires distinct names for every value')
        values = dict(zip(current_pose.joint_names, current_pose.joint_values))
        if any(n not in values for n in names):
            raise ValueError('Starting Configuration is missing arm joints')
        fixed = {**{n: v for n, v in values.items() if n not in names}, **fixed}
        start = [values[n] for n in names]
    else:
        start = list(current_pose) if current_pose is not None else []
        start = start or None
    if start is not None and (len(start) != 6 or not np.isfinite(start).all()):
        raise ValueError('current_pose requires six finite radians')
    if any(n not in joints or n in names or not math.isfinite(v) for n, v in fixed.items()):
        raise ValueError('Invalid fixed joint values')
    solver = kinematics_from_robot(robot, parameters=json_input(parameters), group=group,
        arm_joint_names=names, fixed_joint_values=fixed,
        arm_in_base=as_plane(arm_in_base, units_to_metres) if arm_in_base is not None else None)
    fixed = {**{n: 0. for n in joints if n not in names}, **fixed, **solver.fixed_joint_values}
    for name, value in fixed.items():
        limit = joints[name].limit
        if not math.isfinite(value) or (joints[name].type != 1 and limit is not None and
            ((limit.lower is not None and value < limit.lower) or (limit.upper is not None and value > limit.upper))):
            raise ValueError('Fixed joint value outside URDF limits: '+name)
    ranges = json_input(joint_ranges)
    if ranges is None:
        ranges = [([joints[n].limit.lower, joints[n].limit.upper]
                   if joints[n].limit is not None and joints[n].type != 1 else None) for n in names]
    ranges = normalize_joint_ranges(ranges, 6)
    if start is not None and not _in_ranges(start, ranges):
        raise ValueError('Starting configuration exceeds joint limits')
    step, times, speed, step_limits = _transition_limits(len(targets), start,
        json_input(max_joint_step), json_input(time_intervals), json_input(max_joint_speed))
    options = dict(json_input(collision_options, {}))
    allowed = {'gui', 'allowed_pairs', 'package_paths', 'asset_root', 'ground_z', 'support_links',
               'clearance', 'check_static_self_collisions', 'exclude_gps', 'excluded_collision_links', 'base_collision_model'}
    if set(options)-allowed:
        raise ValueError('Unknown collision options: '+str(sorted(set(options)-allowed)))
    clearance = float(options.get('clearance', 0.))
    if not math.isfinite(clearance) or clearance < 0:
        raise ValueError('clearance must be finite and nonnegative metres')
    if 'ground_z' in options and not math.isfinite(options['ground_z']):
        raise ValueError('ground_z must be finite metres')
    if options.get('support_links') and 'ground_z' not in options:
        raise ValueError('support_links requires ground_z')
    meshes = list(collision_meshes)
    if not collision_check and (meshes or options or scene is not None):
        raise ValueError('Enable collision checking to use obstacle meshes, collision options or a scene')
    if scene is not None and (meshes or set(options)-{'clearance'}):
        raise ValueError('External scene already defines geometry/options; only clearance may be supplied')
    if scene is not None and list(scene.joint_names) != names:
        raise ValueError('Collision scene joint order differs')
    report('geometry')
    tick = perf_counter()
    region = StationaryRegion(targets, solver.arm_in_base, max_distance=1.75,
                              base_height=base_height*units_to_metres, projected=True)
    bases = [as_plane(p, units_to_metres) for p in candidate_planes]
    supplied = bool(bases)
    guesses = []
    spacing = .5 if grid_spacing is None else grid_spacing*units_to_metres
    if not bases:
        bases, guesses, reason = region.candidates(spacing=spacing, yaw_steps=yaw_steps, cancel_check=cancel_check)
        if not bases and not adaptive:
            raise ValueError(reason+' Check target +Z normals, arm mounting/lift height, or split targets into multiple placements.')
    # Polygon coordinates are centred XY arm-origin positions, not footprints.
    polygon, _ = region.polygon(cancel_check=cancel_check)
    placement_region = [[float(x+region.center[0]), float(y+region.center[1]),
                         float(region.height)] for x, y in polygon] if len(polygon) >= 3 else []
    if placement_region:
        placement_region.append(placement_region[0][:])
    timings = dict(geometry_seconds=perf_counter()-tick)
    report('collision_setup', candidates=len(bases))
    with ExitStack() as stack:
        tick = perf_counter()
        world = scene
        if collision_check and world is None:
            constructor = {k: v for k, v in options.items() if k not in ('clearance', 'ground_z', 'support_links')}
            constructor.setdefault('check_static_self_collisions', False)
            constructor.setdefault('exclude_gps', False)
            constructor.setdefault('base_collision_model', 'detailed')
            world = stack.enter_context(PybulletServer(robot=robot, joint_names=names,
                fixed_joint_values=fixed, **constructor))
            for mesh in meshes:
                check_cancel(cancel_check)
                world.add_mesh(mesh, scale=units_to_metres)
            if 'ground_z' in options:
                world.add_ground(options['ground_z'], support_links=options.get('support_links', ()))
        if world is not None:
            world.set_fixed_joints(fixed)
        timings['collision_setup_seconds'] = perf_counter()-tick
        fingerprints = loaded_versions()
        from . import __version__
        settings = dict(package_version=__version__, units_to_metres=units_to_metres,
            search_strategy='adaptive' if adaptive else 'heuristic', requested_search_strategy=search_strategy,
            search_options=search_settings,
            arm_joint_names=names, fixed_joint_values=fixed, joint_ranges=ranges,
            mounting_source=solver.mounting_source, arm_in_base=solver.arm_in_base.to_dict(),
            tcp_in_flange=solver.tool.to_dict(), ur_parameters=list(solver.parameters),
            current_pose=start, rotation_steps=rotation_steps, max_joint_step=step.tolist(),
            max_joint_speed=None if speed is None else speed.tolist(),
            time_intervals=None if times is None else times.tolist(),
            step_limits=None if step_limits is None else step_limits.tolist(),
            build_path=build_path, count_paths=count_paths, fast_validation=bool(fast_validation and not count_paths),
            collision_check=collision_check, clearance=clearance, collision_options=options,
            collision_scene_source='external' if scene is not None else 'owned' if world is not None else 'disabled',
            base_collision_model=getattr(world, 'base_collision_model', None),
            excluded_collision_links=sorted(getattr(world, 'excluded_collision_links', ())),
            allowed_pairs=[sorted(pair) for pair in sorted(getattr(world, 'allowed_pairs', ()), key=lambda p: sorted(p))],
            check_static_self_collisions=getattr(world, 'check_static_self_collisions', None),
            grid_spacing_metres=spacing, base_height_metres=base_height*units_to_metres,
            yaw_steps=yaw_steps, supplied_candidates=supplied, candidate_count=len(bases),
            max_validation_attempts=max_validation_attempts, target_count=len(targets),
            collision_validation='configurations_only' if collision_check else 'not_checked')
        event('stationary.effective_settings', **settings)
        if adaptive:
            from .adaptive_stationary import find_adaptive_stationary_base, footprint_clearance
            search_result = find_adaptive_stationary_base(targets, ik_solver=solver,
                arm_in_base=solver.arm_in_base, candidate_planes=bases if supplied else (),
                current_pose=start, joint_ranges=ranges, periodic=[joints[n].type == 1 for n in names],
                rotation_steps=rotation_steps, max_joint_step=step, step_limits=step_limits,
                build_path=build_path, base_height=base_height*units_to_metres,
                collision=partial(world.is_valid, clearance=clearance) if world else None,
                base_collision=partial(world.is_base_valid, clearance=clearance) if world else None,
                clearance_measure=footprint_clearance(world, targets) if world else None,
                cancel_check=cancel_check, progress=lambda message: report(**message), **search_settings)
            found = search_result.pop('found')
            settings['candidate_count'] = search_result['candidate_count']
            event('stationary.adaptive_search', **search_result)
        else:
            found = find_stationary_base(targets, bases, start, ik_solver=solver,
                objective='heuristic', placement_region=region, build_path=build_path,
                base_collision=partial(world.is_base_valid, clearance=clearance) if world else None,
                collision=partial(world.is_valid, clearance=clearance) if world else None,
                max_validation_attempts=max_validation_attempts, count_paths=count_paths,
                fast_validation=fast_validation, rotation_mode='n_steps', rotation_steps=rotation_steps,
                joint_ranges=ranges, periodic=[joints[n].type == 1 for n in names],
                max_joint_step=step, step_limits=step_limits, cancel_check=cancel_check,
                progress=lambda message: report(**message))
    for diagnostic in found.diagnostics:
        for name, seconds in diagnostic.get('timings', {}).items():
            timings[name] = timings.get(name, 0.)+seconds
    complete = len(found.configurations) == len(targets)
    valid = bool(complete and collision_check)
    output_names = [n for n in joints if n not in names]+names
    configurations = [configuration_from_values(
        [dict(fixed, **dict(zip(names, q)))[n] for n in output_names],
        output_names, [joints[n].type for n in output_names]) for q in found.configurations]
    timings['total_seconds'] = perf_counter()-started
    report('complete', valid=valid, path_complete=complete)
    status=_status(found, len(targets), build_path, collision_check, speed is not None)
    if search_result is not None:
        status=status.replace('positions tested','full validation attempts')
        status+=' Adaptive search: {} proposals, {} fully reachable finalists.'.format(
            search_result['candidate_count'],search_result['stats']['full_checks'])
    result = dict(found=found, targets=targets, initial_guesses=guesses, candidate_count=settings['candidate_count'],
        search_result=search_result,
        placement_region=placement_region,
        configuration_objects=configurations, configurations=found.configurations,
        selected_target_planes=found.selected_target_planes, selected_tcp_rotations=found.selected_tcp_rotations,
        valid=valid, fabrication_validated=valid, path_complete=complete, cancelled=False,
        speed_checked=bool(speed is not None and found.path_search_count),
        collision_check_applied=bool(collision_check), check_edges=False,
        disconnected_detail=found.disconnected_detail, initial_state_failure=found.initial_state_failure,
        unreachable_points=[] if found.base_planes else sorted({i for d in found.diagnostics for i in d.get('unreachable_points', [])}),
        target_diagnostics=found.diagnostics, effective_settings=settings, loaded_modules=fingerprints,
        research_run=str(run.path) if run else None, timings=timings,
        status=status,
        mounting_source=solver.mounting_source, arm_in_base=solver.arm_in_base, fixed_joint_values=fixed)
    event('stationary.plan_result', valid=valid, path_complete=complete,
          disconnected_detail=found.disconnected_detail, timings=timings)
    return result

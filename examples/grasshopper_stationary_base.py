"""Rhino 8 Python 3: target planes -> one stationary robot footprint plane.

Evaluates automatically on every Grasshopper recompute.

Setup
-----
Install motion-toolbox with COMPAS and collision extras in Rhino's Python 3.

Mark optional input parameters as Optional in Grasshopper so empty inputs do not
prevent the component from running. current_pose and arm_in_base may also be
removed from the component inputs entirely.

Required inputs
---------------
  target_planes    List, Plane: ordered world TCP targets; +Z points away from robot
  robot            Item: COMPAS/MobileRobot with attached active tool (metre model)

Optional inputs (defaults)
--------------------------
Arm and mounting:
  current_pose     List, float: six starting arm angles; empty excludes approach motion
  arm_in_base      Item, Plane: override; otherwise offline calibration or URDF controller base
  arm_joint_names List, str: optional override; inferred from robot/tool chain by default
  fixed_joint_values Item, JSON: nonplanned joints, e.g. {"lift": 0.2}, metres/radians
  ur_parameters   Item, JSON: optional six UR dimensions; default UR20
  group           Item, str: active tool group, if needed

Placement and units:
  candidate_planes List, Plane: optional exclusive footprint set, still checked against placement rules
  units_to_metres  Item, float (1): supplied planes/meshes/search lengths use these units
  grid_spacing     Item, float (0.5 / units_to_metres): maximum grid spacing
  base_height      Item, float (0): footprint world Z
  yaw_steps        Item, int (4): evenly spaced footprint orientations
  max_validation_attempts Item, int (3): maximum final-position IK checks;
                   stop at first success

Arm validation and path:
  rotation_steps   Item, int (8): TCP-Z orientation samples (1 fixes orientation)
  max_joint_step   Item, float (2.5): radians per arm joint per target step
  build_path       Item, bool (True): build one joint path for the selected base only
  count_paths      Item, bool (False): also count every possible joint path (slower)
  collision_meshes List, Mesh: wall and environment obstacles
                   (needed for full-body wall clearance)
  collision_check Item, bool (True): model/tool/environment at target configurations only

Toolbox loading:
  toolbox_src     Item, str: motion_toolbox/src; restart Rhino when switching installations

Outputs
-------
Placement:
  base_plane: one Rhino footprint plane, None on failure.
  initial_base_plane: first base-body-clear geometry guess, even if arm
                      validation fails.
  initial_guesses: geometry-valid Rhino footprint seeds;
                   IK/collisions not yet certified.
  standoff: minimum signed distance behind all target planes, metres.
  max_target_distance: farthest target from the calibrated arm-base origin,
                       metres.

Arm results:
  joint_plan: DataTree, one branch per target.
  path_cost: joint-path cost.
  path_count: exact integer text when count_paths=True; otherwise "not counted".
  solution_counts: feasible configurations per target at the selected base.
  ik_option_count: product of those configuration counts, as text.

Diagnostics and timing:
  status, diagnostics: summary and detailed results.
  candidate_count: number of geometry candidates.
  path_search_count: 0 or 1; complete-path searches performed.
  validation_attempts: number of expensive arm validations.
  base_collision_checks: number of base-only tests.
  elapsed_seconds: total component execution time.
  timings: seconds spent in geometry, collision setup, IK, joint expansion,
           collision checking and path search.

Search behavior
---------------
No robot motion is commanded. Search assumes a horizontal floor.

It checks every supplied/generated candidate geometrically and favors maximum
ground-plane wall
standoff within the common 1.75 m XY reach disks and negative-Z-side region.

Base-body/environment checks require no arm configuration. Only the selected
position gets full target IK and configuration collision checks. Failed validation
can try a bounded number of other positions; the first success wins, not the
highest IK count over every base. Only that winner gets a joint path when requested.
No collision checks occur between configurations.

The arm-base origin must be strictly on the negative-Z side of EVERY projected
target and at most 1.75 m from EVERY target origin in XY. Final IK checks actual
3D reach. TCP-Z rotation preserves the normals.

Seeds maximize standoff inside the common reach/negative-side region; the search
also samples inward positions, its boundary and interior. No side-flip toggle,
guess_distance or search_margin is needed. Old inputs with those names are ignored.

After a collision, try other headings before moving the shoulder position.
After an IK failure, prefer inward positions. Retries check failed targets first;
the final joint plan and solution_counts remain in the original target order.

Counts refer to sampled rotations/IK branches, not continuous motion alternatives.
A finite grid can miss feasible positions; no continuous-space optimum is claimed.

Mounting frame
--------------
Controller base may differ from URDF base_link. Fixed lift and arm_in_base must agree.
arm_in_base locates the arm's controller origin/axes relative to the footprint.
For example, an origin of (0, 0, 0.8) with world-XY axes describes an arm mounted
0.8 metres above the footprint with aligned axes (use 800 for millimetre inputs).
This is the fixed mounting relationship, not the robot's world placement or its
six joint angles. Leave it empty to use the robot's stored mounting calibration.

When calibration is missing, the URDF controller base frame and upstream lift
configuration are used. No live ROS lookup or identity mounting frame is assumed.
"""
import sys
from contextlib import ExitStack
from decimal import Decimal
from time import perf_counter


def _input(name, default=None):
    value = globals().get(name)
    return default if value is None else value


base_plane, joint_plan, path_cost = None, None, None
diagnostics, candidate_count, status = [], 0, ''
path_count, solution_counts = '0', []
ik_option_count, path_search_count = '0', 0
initial_guesses = []
initial_base_plane = None
validation_attempts, base_collision_checks = 0, 0
timings = {}
standoff, max_target_distance = None, None
toolbox_loaded_from = ''
started_at = perf_counter()
try:
    import importlib
    import inspect
    from pathlib import Path
    source = _input('toolbox_src')
    if source:
        source = Path(str(source)).expanduser().resolve()
        if not (source / 'motion_toolbox' / '__init__.py').is_file():
            raise ValueError('toolbox_src must point to the src directory containing motion_toolbox/__init__.py: ' + str(source))
        loaded = sys.modules.get('motion_toolbox')
        if loaded is not None and Path(loaded.__file__).resolve().parent != source / 'motion_toolbox':
            raise RuntimeError('Rhino has cached a different toolbox at {}. Restart Rhino with toolbox_src set to {}.'.format(
                loaded.__file__, source))
        if str(source) in sys.path:
            sys.path.remove(str(source))
        sys.path.insert(0, str(source))
    import motion_toolbox.robot_adapter as adapter
    toolbox_loaded_from = str(Path(adapter.__file__).resolve())
    required = {'fixed_joint_values', 'arm_joint_names'}
    if (getattr(adapter, 'STATIONARY_ADAPTER_VERSION', 0) < 4 or
        not required.issubset(inspect.signature(adapter.kinematics_from_robot).parameters)):
        # Rhino retains Python modules between component recomputes. Refresh the
        # adapter only when its cached API is older than this component requires.
        importlib.invalidate_caches()
        adapter = importlib.reload(adapter)
    if (getattr(adapter, 'STATIONARY_ADAPTER_VERSION', 0) < 4 or
        not required.issubset(inspect.signature(adapter.kinematics_from_robot).parameters)):
        raise RuntimeError('Outdated toolbox file: {}. Set toolbox_src to the updated motion_toolbox/src directory and restart Rhino.'.format(
            toolbox_loaded_from))
    # Reload the planning dependency chain together: refreshing only the adapter
    # leaves old imported functions in base_planning/planning alive in Rhino.
    pipeline_names = (
        'motion_toolbox.kinematics.ur', 'motion_toolbox.kinematics.solver',
        'motion_toolbox.graph', 'motion_toolbox.planning',
        'motion_toolbox.base_planning', 'motion_toolbox.stationary_region',
        'motion_toolbox.robot_planning',
    )
    pipeline = [importlib.import_module(name) for name in pipeline_names]
    def source_stamp(module):
        path = Path(module.__file__).resolve()
        stat = path.stat()
        return str(path), stat.st_mtime_ns, stat.st_size
    base_module = pipeline[4]
    planner_arguments = {'objective', 'placement_region', 'build_path', 'base_collision', 'max_validation_attempts'}
    stale = any(getattr(module, '_stationary_loaded_stamp', None) != source_stamp(module)
                for module in pipeline)
    stale = stale or not planner_arguments.issubset(inspect.signature(base_module.find_stationary_base).parameters)
    stale = stale or 'path_count' not in getattr(pipeline[2].GraphResult, '__dataclass_fields__', {})
    if stale:
        importlib.invalidate_caches()
        for module in pipeline:
            importlib.reload(module)
            module._stationary_loaded_stamp = source_stamp(module)
    if (not planner_arguments.issubset(inspect.signature(base_module.find_stationary_base).parameters)
        or 'path_count' not in getattr(pipeline[2].GraphResult, '__dataclass_fields__', {})):
        raise RuntimeError('Outdated planner at {}. Set toolbox_src to the updated motion_toolbox/src directory and restart Rhino.'.format(
            base_module.__file__))
    from motion_toolbox.geometry import as_plane, to_rhino
    from motion_toolbox.base_planning import find_stationary_base
    from motion_toolbox.stationary_region import StationaryRegion
    from motion_toolbox.robot_adapter import kinematics_from_robot, resolve_arm_joint_names
    from motion_toolbox.robot_planning import json_input
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path
    scale = _input('units_to_metres', 1.0)
    import math
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('units_to_metres must be positive; use 0.001 for millimetre inputs')
    model = _input('robot')
    if model is None:
        raise ValueError('Connect robot: its arm geometry, tool and collision model are required')
    targets = [as_plane(p, scale) for p in _input('target_planes', [])]
    bases = [as_plane(p, scale) for p in _input('candidate_planes', [])]
    names = resolve_arm_joint_names(model, _input('arm_joint_names'), group=_input('group'))
    joints = {j.name: j for j in model.model.get_configurable_joints()}
    if len(names) != 6 or len(set(names)) != 6 or any(n not in joints or joints[n].type not in (0,1) for n in names):
        raise ValueError('Provide six distinct UR revolute arm_joint_names in analytic order')
    seed = list(_input('current_pose', [])) or None
    if seed is not None and len(seed) != 6:
        raise ValueError('Provide six starting arm angles or leave current_pose empty')
    arm = _input('arm_in_base')
    solver = kinematics_from_robot(model, arm_in_base=as_plane(arm, scale) if arm is not None else None,
        parameters=json_input(_input('ur_parameters')), group=_input('group'),
        fixed_joint_values=json_input(_input('fixed_joint_values'), {}), arm_joint_names=names)
    geometry_started = perf_counter()
    region = StationaryRegion(targets, solver.arm_in_base,
        max_distance=1.75, base_height=_input('base_height', 0)*scale, projected=True)
    if not bases:
        bases, guesses, reason = region.candidates(
            spacing=_input('grid_spacing', .5/scale)*scale, yaw_steps=_input('yaw_steps', 4))
        initial_guesses = [to_rhino(p, 1/scale) for p in guesses]
        if not bases:
            raise ValueError(reason + ' Check target +Z normals, arm mounting/lift height, or split targets into multiple placements.')
    candidate_count = len(bases)
    timings['geometry_seconds'] = perf_counter()-geometry_started
    periodic = [joints[n].type == 1 for n in names]
    ranges = [([joints[n].limit.lower, joints[n].limit.upper]
               if joints[n].limit is not None and joints[n].type != 1 else None) for n in names]
    meshes = list(_input('collision_meshes', []))
    enabled = _input('collision_check', True)
    if meshes and not enabled:
        raise ValueError('Enable collision checking to use obstacle meshes')
    with ExitStack() as stack:
        collision_setup_started = perf_counter()
        scene = None
        if enabled:
            import motion_toolbox.collision as collision_module
            if getattr(collision_module, 'COLLISION_API_VERSION', 0) < 8:
                collision_module = importlib.reload(collision_module)
            if getattr(collision_module, 'COLLISION_API_VERSION', 0) < 8:
                raise RuntimeError('Outdated collision module: ' + str(collision_module.__file__))
            PybulletServer = collision_module.PybulletServer
            scene = stack.enter_context(PybulletServer(robot=model, joint_names=names,
                check_static_self_collisions=False))
            scene.set_fixed_joints(solver.fixed_joint_values)
            for mesh in meshes:
                scene.add_mesh(mesh, scale=scale)
        timings['collision_setup_seconds'] = perf_counter()-collision_setup_started
        found = find_stationary_base(targets, bases, seed, ik_solver=solver,
            objective='heuristic', placement_region=region, build_path=_input('build_path', True),
            base_collision=scene.is_base_valid if scene else None,
            max_validation_attempts=_input('max_validation_attempts', 3),
            count_paths=_input('count_paths', False),
            collision=scene.is_valid if scene else None,
            rotation_mode='n_steps', rotation_steps=_input('rotation_steps', 8),
            joint_ranges=ranges, periodic=periodic, max_joint_step=_input('max_joint_step', 2.5))
    path_cost = found.cost if found.path_search_count else None
    for diagnostic in found.diagnostics:
        for name, seconds in diagnostic.get('timings', {}).items():
            timings[name] = timings.get(name, 0.0)+seconds
    validation_attempts, base_collision_checks = found.validation_attempts, found.base_collision_checks
    initial_base_plane = to_rhino(found.heuristic_plane, 1/scale) if found.heuristic_plane is not None else None
    path_search_count, ik_option_count = found.path_search_count, str(Decimal(found.ik_option_count))
    diagnostics = ['Base {}: {}; IK checked {}; reachable {} of {} checked targets ({} total); solutions {}; path checked {}; '
        'standoff {:.3f} m; farthest 3D {:.3f} m; wrong-side targets {}; too-far XY targets {}'.format(
        i, d['reason'], d['ik_checked'], d['reachable_targets'], d['targets_checked'], len(targets),
        '{}-{} per target'.format(min(d['solution_counts']), max(d['solution_counts'])) if d['solution_counts'] else 'none',
        d['path_checked'], d['standoff'], d['max_target_distance'],
        d['wrong_side_points'], d['too_far_points'])
        for i, d in enumerate(found.diagnostics)]
    diagnostics.insert(0, 'Mounting: {}; arm origin in footprint {} m; fixed joints {}'.format(
        solver.mounting_source, solver.arm_in_base.origin.tolist(), solver.fixed_joint_values))
    diagnostics.insert(0, 'Toolbox loaded from: ' + toolbox_loaded_from)
    failure_details = []
    for attempt, diagnostic in enumerate(found.diagnostics):
        if diagnostic.get('base_collision_detail'):
            failure_details.append('Candidate {}: {}'.format(attempt, diagnostic['base_collision_detail']))
        detail = diagnostic.get('failed_target_details')
        if detail is None:
            continue
        rejected = sorted(detail['rejection_reasons'].items(), key=lambda item: -item[1])
        text = ('Candidate {} target {}: {} raw IK, {} within joint limits, {} collision-free. {}').format(
            attempt, detail['target_index'], detail['raw_ik'], detail['within_joint_limits'],
            detail['collision_free'], '; '.join('{} ({} rejections)'.format(reason, count) for reason, count in rejected[:3]))
        failure_details.append(text)
    diagnostics.extend(failure_details)
    joint_plan = DataTree[float]()
    if found.base_planes:
        base_plane = to_rhino(found.base_plane, 1/scale)
        # Decimal preserves exact text beyond Python's integer-string digit limit.
        path_count = str(Decimal(found.path_count)) if _input('count_paths', False) else 'not counted'
        solution_counts = found.candidate_counts
        standoff, max_target_distance = found.standoff, found.max_target_distance
        for i,q in enumerate(found.configurations):
            for v in q:
                joint_plan.Add(float(v), GH_Path(i))
        status = 'Found base. All {} targets reachable; {} position{} tested; wall distance {:.2f} m.'.format(
            len(targets), validation_attempts, '' if validation_attempts == 1 else 's', standoff)
        if not enabled:
            status += ' Collision checks off.'
        if found.path_search_count and not found.configurations:
            status += ' Joint path blocked by max_joint_step.'
    else:
        from collections import Counter
        failed = [d['failed_target_details'] for d in found.diagnostics if d.get('failed_target_details')]
        status = 'No valid base. {} positions tested.'.format(validation_attempts)
        if failed:
            indices = sorted({d['target_index']+1 for d in failed})
            status += ' Failed at target{} {} of {}.'.format('s' if len(indices)>1 else '',
                ', '.join(map(str, indices)), len(targets))
            blockers = Counter()
            for detail in failed:
                blockers.update(detail['rejection_reasons'])
            if blockers:
                status += ' Main collisions: ' + '; '.join(reason.replace('robot_arm_', '').replace('robot_', '')
                    for reason, _ in blockers.most_common(2)) + '.'
            else:
                status += ' IK or joint limits rejected the configurations.'
        else:
            status += ' Placement constraints or base collisions rejected the candidates.'
        status += ' See diagnostics for details.'
except Exception as error:
    base_plane, joint_plan, path_cost = None, None, None
    path_count, solution_counts = '0', []
    ik_option_count, path_search_count = '0', 0
    standoff, max_target_distance = None, None
    status = '{}: {}'.format(type(error).__name__, error)
    diagnostics = [status]
    if toolbox_loaded_from:
        diagnostics.append('Toolbox loaded from: ' + toolbox_loaded_from)

elapsed_seconds = perf_counter()-started_at
diagnostics.append('Timing: ' + '; '.join('{} {:.2f}s'.format(name.removesuffix('_seconds'), seconds)
    for name, seconds in timings.items()))
status += ' {:.1f} s.'.format(elapsed_seconds)
if base_plane is None:
    print(status)
    # Show failures on the component even when the status output is not wired.
    if 'ghenv' in globals():
        from Grasshopper.Kernel import GH_RuntimeMessageLevel
        ghenv.Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, status)

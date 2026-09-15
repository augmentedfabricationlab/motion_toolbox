"""Complete robot-object planning workflow shared by scripts and Grasshopper."""
from motion_toolbox.recording import recorded
from contextlib import ExitStack
from functools import partial
import json
import math
from time import perf_counter
from .geometry import Plane, as_plane
from .robot_adapter import kinematics_from_robot, configuration_from_values, resolve_arm_joint_names, _active_tool
from .planning import calculate_partial_trajectory

ROBOT_COMPONENT_VERSION = 5


def json_input(value, default=None):
    return default if value is None or (isinstance(value, str) and not value.strip()) else json.loads(value) if isinstance(value, str) else value


@recorded
def plan_robot(robot, targets, bases=None, current_pose=None, arm_in_base=None, arm_joint_names=None,
               collision_meshes=(), model_units_to_metres=1.0, *,
               collision_check=True, check_edges=False, rotation_steps=24,
               joint_ranges=None, max_joint_step=2.5, fixed_joint_values=None,
               collision_options=None, group=None, parameters=None, scene=None):
    """Plan all targets and return joint rows, named configurations and diagnostics.

    Input planes/meshes use model_units_to_metres. Robot model, active tool and
    fixed_joint_values must already use metres/radians. bases are footprints;
    arm_in_base is the UR controller base in footprint coordinates, including
    lift/mounting rotation (not necessarily URDF base_link). If
    omitted it is read from offline calibration or the URDF controller link.
    current_pose is optional; without it the first target has no start constraint.
    Collisions are checked at configurations; check_edges enables sampled edges.
    An externally supplied scene is reused and never closed or populated here.
    """
    started = perf_counter()
    if robot is None:
        raise ValueError('Connect a robot object')
    warnings = []
    if _active_tool(robot, group) is None:
        warnings.append('No tool is attached for the selected planning group. '
                        'Targets are interpreted as flange/tool0 poses with no TCP offset.')
    if not math.isfinite(model_units_to_metres) or model_units_to_metres <= 0:
        raise ValueError('model_units_to_metres must be positive')
    targets = [as_plane(p, model_units_to_metres) for p in targets]
    if not targets:
        raise ValueError('Connect at least one target plane using List access')
    names = resolve_arm_joint_names(robot, arm_joint_names, group=group)
    model_joints = {j.name: j for j in robot.model.get_configurable_joints()}
    if len(names) != 6 or len(set(names)) != 6 or any(n not in model_joints for n in names):
        raise ValueError('arm_joint_names must contain six distinct UR arm joints in analytic IK order')
    if any(model_joints[n].type not in (0, 1) for n in names):
        raise ValueError('The UR analytic solver requires six revolute/continuous arm joints')
    all_values = {n: 0.0 for n in model_joints}
    supplied_fixed = {}
    if hasattr(current_pose, 'joint_values'):
        given = dict(zip(current_pose.joint_names, current_pose.joint_values))
        if any(n not in given for n in names):
            raise ValueError('Starting Configuration is missing selected arm joints')
        all_values.update({n: float(v) for n, v in given.items() if n in all_values})
        supplied_fixed = {n: all_values[n] for n in given if n in model_joints and n not in names}
        start = [all_values[n] for n in names]
    else:
        start = [float(v) for v in current_pose] if current_pose is not None else []
        start = start or None
    if start is not None and (len(start) != 6 or not all(math.isfinite(v) for v in start)):
        raise ValueError('current_pose must contain six finite joint angles in radians')
    fixed = dict(json_input(fixed_joint_values, {}))
    if any(n not in model_joints or n in names for n in fixed):
        raise ValueError('fixed_joint_values must name nonplanned joints from the robot model')
    all_values.update(fixed)
    supplied_fixed.update(fixed)
    for n, v in all_values.items():
        if not math.isfinite(v):
            raise ValueError('Nonfinite joint value: ' + n)
    base_items = list(bases) if bases is not None else []
    if base_items:
        converted_bases = [as_plane(b, model_units_to_metres) for b in base_items]
    else:
        bcf = getattr(robot, 'BCF', None)
        if bcf is None:
            raise ValueError('Provide base_planes or initialize robot.BCF')
        converted_bases = [as_plane(bcf)]  # robot properties are already in metres
    if len(converted_bases) not in (1, len(targets)):
        raise ValueError('Provide one footprint base plane or one per target')
    solver = kinematics_from_robot(robot, parameters=json_input(parameters), group=group,
        arm_in_base=as_plane(arm_in_base, model_units_to_metres) if arm_in_base is not None else None,
        arm_joint_names=names, fixed_joint_values=supplied_fixed)
    all_values.update(solver.fixed_joint_values)
    ranges = json_input(joint_ranges)
    if ranges is None:
        ranges = [([model_joints[n].limit.lower, model_joints[n].limit.upper]
                   if model_joints[n].limit is not None and model_joints[n].type != 1 else None) for n in names]
    periodic = [model_joints[n].type == 1 for n in names]
    settings = dict(json_input(collision_options, {}))
    allowed = {'gui', 'allowed_pairs', 'package_paths', 'asset_root', 'ground_z', 'support_links',
               'joint_resolution', 'base_resolution', 'yaw_resolution', 'clearance', 'check_static_self_collisions'}
    unknown = set(settings)-allowed
    if unknown:
        raise ValueError('Unknown collision options: ' + ', '.join(sorted(unknown)))
    meshes = list(collision_meshes) if collision_meshes is not None else []
    if not collision_check and (meshes or scene is not None):
        raise ValueError('Enable collision_check to use supplied meshes or scene')
    with ExitStack() as stack:
        world = None
        if collision_check:
            if scene is not None:
                if meshes or settings:
                    raise ValueError('Configure an external scene before passing it; omit meshes and collision_options')
                world = scene
                if list(world.joint_names) != names:
                    raise ValueError('Scene joint order must match arm_joint_names')
            else:
                from .collision import PybulletServer
                constructor = {k: settings[k] for k in ('gui', 'allowed_pairs', 'package_paths', 'asset_root', 'check_static_self_collisions') if k in settings}
                constructor.setdefault('check_static_self_collisions', False)
                world = stack.enter_context(PybulletServer(robot=robot, joint_names=names, **constructor))
                for mesh in meshes:
                    world.add_mesh(mesh, scale=model_units_to_metres)
                if 'ground_z' in settings:
                    world.add_ground(settings['ground_z'], support_links=settings.get('support_links', ()))
            world.set_fixed_joints({n: v for n, v in all_values.items() if n not in names})
        clearance = settings.get('clearance', 0.0)
        edge_options = {k: settings[k] for k in ('joint_resolution', 'base_resolution', 'yaw_resolution') if k in settings}
        setup_seconds = perf_counter()-started
        result = calculate_partial_trajectory(start, targets, base_planes=converted_bases,
            ik_solver=solver, rotation_mode='n_steps', rotation_steps=rotation_steps,
            joint_ranges=ranges, periodic=periodic, max_joint_step=max_joint_step,
            collision=partial(world.is_valid, clearance=clearance) if world else None,
            transition_check=partial(world.edge_is_valid, periodic=periodic, clearance=clearance, **edge_options) if world and check_edges else None)
    # Export the lift and planned arm only. Wheel joints remain part of the
    # collision world, but do not belong in the fabrication configuration.
    output_names = [n for n, joint in model_joints.items() if joint.type == 2 and n not in names] + names
    types = [model_joints[n].type for n in output_names]
    objects = []
    for q in result['configurations']:
        values = dict(all_values, **dict(zip(names, q)))
        objects.append(configuration_from_values([values[n] for n in output_names], output_names, types))
    result['configuration_objects'] = objects
    result['warnings'] = warnings
    result['mounting_source'] = solver.mounting_source
    result['arm_in_base'] = solver.arm_in_base
    result['fixed_joint_values'] = {n: v for n, v in all_values.items() if n not in names}
    result['base_planes'] = converted_bases*len(targets) if len(converted_bases) == 1 else converted_bases
    result['timings']['setup_seconds'] = setup_seconds
    result['timings']['total_seconds'] = perf_counter()-started
    return result

"""Paste into a Rhino 8 Python 3 component. Recomputes automatically.

Inputs:
  robot          Item: MobileRobot / COMPAS FAB Robot with its active TCP tool
  current_pose   Item: named COMPAS Configuration (including lift if used).
                 Alternatively use List access for six arm angles in radians.
  target_planes  List, Plane: world TCP planes, used exactly as supplied
Optional inputs (may be omitted):
  model_units_to_metres Item, float: 1 for metres, .001 for millimetres
  group          Item, str: active tool group when multiple tools are attached
  arm_in_base    Item, Plane: controller-base override in footprint coordinates
  toolbox_src    Item, str: development src directory; normally omit
Outputs:
  configurations List: one named Configuration per target; None for failures
  failed_indices List: zero-based indices with no matching, in-limit IK
  status         Text: success count / input error

Uses robot.BCF, the active TCP, and offline mounting/calibration (UR20 nominal
geometry by default). Robot properties are in metres; supplied planes use the
scale input. Keep the robot's offline lift/mounting state consistent with the
starting configuration. Matches its shoulder/elbow/wrist branch at EVERY target.
Selects the solution nearest the ORIGINAL start, independently for each plane.
Equivalent full turns are fitted inside joint limits near the starting angles.
No target rotations, base search, graph, collision checks or transition checks.
These are IK matches, not a validated motion path. Requires motion_toolbox and
COMPAS FAB 1.x installed in Rhino's Python 3 environment.
"""
import math
import sys


def match_ik(robot, current_pose, targets, *, model_units_to_metres=1.,
             group=None, arm_in_base=None):
    from motion_toolbox.geometry import as_plane
    from motion_toolbox.robot_adapter import (
        _active_tool, resolve_arm_joint_names, kinematics_from_robot,
        configuration_from_values,
    )

    if robot is None or current_pose is None:
        raise ValueError('Connect robot and current_pose')
    if not math.isfinite(model_units_to_metres) or model_units_to_metres <= 0:
        raise ValueError('model_units_to_metres must be positive')
    if _active_tool(robot, group) is None:
        raise ValueError('Attach the active TCP tool to robot')
    if getattr(robot, 'BCF', None) is None:
        raise ValueError('Initialize robot.BCF (the fixed footprint plane)')
    names = resolve_arm_joint_names(robot, group=group)
    joints = {j.name: j for j in robot.model.get_configurable_joints()}
    values = {n: 0. for n in joints}
    if hasattr(current_pose, 'joint_values'):
        given = dict(zip(current_pose.joint_names, current_pose.joint_values))
        if any(n not in given for n in names):
            raise ValueError('Starting Configuration must name all six arm joints')
        values.update({n: float(v) for n, v in given.items() if n in joints})
        start = [values[n] for n in names]
        fixed = {n: values[n] for n in given if n in joints and n not in names}
    else:
        start = [float(v) for v in current_pose]
        fixed = {}
    if len(start) != 6 or not all(math.isfinite(v) for v in start + list(values.values())):
        raise ValueError('current_pose must contain six finite arm angles in radians')
    limits = [(joints[n].limit if joints[n].type != 1 else None) for n in names]
    for value, limit in zip(start, limits):
        if limit is not None and not limit.lower <= value <= limit.upper:
            raise ValueError('Starting arm configuration is outside joint limits')
    solver = kinematics_from_robot(robot, group=group, arm_joint_names=names,
        fixed_joint_values=fixed,
        arm_in_base=as_plane(arm_in_base, model_units_to_metres) if arm_in_base is not None else None)
    values.update(solver.fixed_joint_values)
    branch = solver.configuration_branch(start)
    if branch is None:
        raise ValueError('Starting configuration is on an ambiguous IK branch boundary')
    base = as_plane(robot.BCF)
    output_names = [n for n, j in joints.items() if j.type == 2 and n not in names] + names
    output_types = [joints[n].type for n in output_names]
    configurations, failed = [], []
    for index, target in enumerate(targets):
        candidates = []
        for raw in solver(as_plane(target, model_units_to_metres), base):
            if solver.configuration_branch(raw) != branch:
                continue
            q = []
            for angle, reference, limit in zip(raw, start, limits):
                turns = round((reference-angle)/(2*math.pi))
                if limit is not None:
                    low = math.ceil((limit.lower-angle)/(2*math.pi))
                    high = math.floor((limit.upper-angle)/(2*math.pi))
                    if low > high:
                        break
                    turns = min(high, max(low, turns))
                q.append(float(angle + turns*2*math.pi))
            if len(q) == 6:
                candidates.append(q)
        if not candidates:
            configurations.append(None)
            failed.append(index)
            continue
        q = min(candidates, key=lambda row: sum((a-b)**2 for a, b in zip(row, start)))
        state = dict(values, **dict(zip(names, q)))
        configurations.append(configuration_from_values(
            [state[n] for n in output_names], output_names, output_types))
    return configurations, failed


configurations, failed_indices = [], []
status = ''
try:
    source = globals().get('toolbox_src')
    if source and str(source) not in sys.path:
        sys.path.insert(0, str(source))
    scale = globals().get('model_units_to_metres')
    configurations, failed_indices = match_ik(
        globals().get('robot'), globals().get('current_pose'),
        list(globals().get('target_planes') or []),
        model_units_to_metres=1. if scale is None else scale,
        group=globals().get('group'), arm_in_base=globals().get('arm_in_base'))
    status = 'Matched {}/{} targets. IK only; collisions and transitions unchecked.'.format(
        len(configurations)-len(failed_indices), len(configurations))
except Exception as error:
    configurations, failed_indices = [], []
    status = '{}: {}'.format(type(error).__name__, error)

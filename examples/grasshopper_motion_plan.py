"""Ready-to-paste Rhino 8 Python 3 component: ordered TCP planes -> joint path.

Evaluates automatically on every Grasshopper recompute.

Required inputs:
  target_planes   List access, Plane: ordered TCP target planes in world coordinates
  base_plane     Item access, Plane: UR controller base frame in world coordinates
  tcp_plane      Item access, Plane: TCP expressed in the flange frame
Optional inputs:
  current_pose   List access, float: six starting arm angles; empty has no start constraint
  rotation_steps Item access, int: 1 fixes orientation, 24 samples a full turn
  units_to_metres Item access, float: 1 for metres, 0.001 for millimetres
  max_joint_step Item access, float: maximum per-joint step in radians (default 2.5)
  joint_ranges   Item access: JSON [[min,max], ...], radians; optional
  toolbox_src    Item access, str: path to the new motion_toolbox/src directory

Outputs:
  joint_path          DataTree: branch {i} contains six joint angles for target i
  configurations      Nested lists of the same joint angles (standalone output)
  path_cost           Sum of joint-space edge lengths, including starting pose
  unreachable_points  Zero-based target indices without an IK solution
  status              Outcome / input error

Uses the UR20 analytic model. No collision checks in this minimal component.
Install motion-toolbox (and NumPy) in the component's Python environment, or
provide toolbox_src when NumPy is already available. Plane origins, including
the flange-local TCP origin, use the selected model units.
The controller base is not necessarily URDF base_link; the source UR20 model
has a 180-degree Z rotation between these frames. TCP is relative to tool0.
"""
import json
import sys


def plan_planes(targets, current_pose, base_plane, tcp_plane, *, rotation_steps=1,
                units_to_metres=1.0, max_joint_step=2.5, joint_ranges=None):
    from motion_toolbox.geometry import Plane, as_plane
    from motion_toolbox.planning import calculate_partial_trajectory
    current_pose = list(current_pose) if current_pose is not None else []
    if current_pose and len(current_pose) != 6:
        raise ValueError('current_pose must contain six joint angles in radians')
    if not targets:
        raise ValueError('Connect target_planes using List access')
    if base_plane is None or tcp_plane is None:
        raise ValueError('Connect base_plane and tcp_plane; use World XY explicitly for identity')
    if isinstance(joint_ranges, str):
        joint_ranges = json.loads(joint_ranges)
    steps = int(rotation_steps)
    if steps < 1 or steps != rotation_steps:
        raise ValueError('rotation_steps must be a positive integer')
    return calculate_partial_trajectory(
        current_pose or None, [as_plane(p, units_to_metres) for p in targets],
        base_planes=[as_plane(base_plane, units_to_metres)],
        tool=as_plane(tcp_plane, units_to_metres),
        arm_in_base=Plane.world_xy(),
        rotation_mode='n_steps' if steps > 1 else False, rotation_steps=steps,
        max_joint_step=max_joint_step, joint_ranges=joint_ranges,
    )


joint_path = None
configurations = []
path_cost = None
unreachable_points = []
status = ''
result = None

try:
    source = globals().get('toolbox_src')
    if source and str(source) not in sys.path:
        sys.path.insert(0, str(source))
    result = plan_planes(
        list(globals().get('target_planes') or []),
        list(globals().get('current_pose') or []),
        globals().get('base_plane'), globals().get('tcp_plane'),
        rotation_steps=globals().get('rotation_steps') if globals().get('rotation_steps') is not None else 1,
        units_to_metres=globals().get('units_to_metres') if globals().get('units_to_metres') is not None else 1.0,
        max_joint_step=globals().get('max_joint_step') if globals().get('max_joint_step') is not None else 2.5,
        joint_ranges=globals().get('joint_ranges'),
    )
    configurations = result['configurations']
    path_cost = result['path_length']
    unreachable_points = result['unreachable_points']
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path
    joint_path = DataTree[float]()
    for i, q in enumerate(configurations):
        for value in q:
            joint_path.Add(float(value), GH_Path(i))
    if configurations:
        status = 'Planned {} targets. IK + shortest path; collision checking off.'.format(len(configurations))
    elif unreachable_points:
        status = 'No complete path. Targets without IK: {}'.format(unreachable_points)
    else:
        status = 'IK exists, but no connected path satisfies the joint-step limit.'
except Exception as error:
    joint_path, configurations, path_cost, result, unreachable_points = None, [], None, None, []
    status = '{}: {}'.format(type(error).__name__, error)

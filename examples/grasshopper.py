"""Runnable Rhino 8 Python 3 component: Robot + TCP planes -> optimum joint plan.

Evaluates automatically on every Grasshopper recompute.
Mark optional inputs as Optional in Grasshopper, or remove unused inputs.
Uses the installed motion_toolbox automatically and refreshes stale planner imports.
Missing active tools produce a Grasshopper warning; planning uses flange/tool0 targets.

Inputs (names must match; all optional inputs may be omitted):
  robot                 Item: MobileRobot / COMPAS FAB Robot, model in metres
  target_planes         List, Plane: ordered world TCP target planes
  base_planes           List, Plane: one footprint or one per target; empty uses robot.BCF
  current_pose          List, float: optional six starting arm angles in radians
  arm_in_base           Item, Plane: UR controller base in footprint INCLUDING lift/rotation
                        Optional override; inferred from offline calibration or URDF.
Optional:
  arm_joint_names List, str: optional override; inferred from robot/tool chain by default
  collision_meshes      List, Mesh: environment obstacles
  model_units_to_metres Item, float: 1 for metres, 0.001 for millimetres (default 1)
  rotation_steps        Item, int: 1 fixes TCP orientation; default 24 samples a turn
  collision_check       Item, bool: default True (also self/tool collisions)
  check_edges           Item, bool: default False; opt in to sampled edge checks
  joint_ranges          Item: JSON [[min,max],...]; defaults to model limits
  max_joint_step        Item, float: default 2.5 radians per joint per step
  fixed_joint_values    Item: JSON {"lift_joint_name": 0.2, ...}, metres/radians
  collision_options     Item: JSON options (see motion_toolbox.robot_planning)
  collision_scene       Item: optional preconfigured PybulletServer to reuse
  group                 Item, str: tool/planning group when more than one is present
  ur_parameters         Item: optional JSON list of six UR geometry parameters
  toolbox_src           Item, str: optional development override; normally omit

Outputs:
  joint_plan            DataTree: six arm joint values in branch {target_index}
  configurations        List of named COMPAS Configurations: lift, then six arm joints
  base_result           List of footprint planes actually used, one per target
  path_cost             Total joint-space cost
  unreachable_points    Zero-based indices without feasible IK
  result                Full diagnostic dictionary, or None on error
  status                Outcome with collision checking state
  timings               Dictionary: setup, IK, joint expansion, collisions and graph seconds
  diagnostics           List of text: warnings, failed targets, filter counts and rejection reasons
  version               String: loaded motion-toolbox package version

Paste this entire file into a Python 3 component. Install NumPy and COMPAS FAB
1.x in that Python environment, plus PyBullet for collisions. Robot/tool geometry
is already in metres; the scale input applies only to supplied planes/meshes.
This uses UR20 geometry by default. It does not search for new base locations.
The controller base can differ from URDF base_link (by Z=180 degrees in the source
UR20 model). Match this frame to your model; the active tool is relative to tool0.
"""
import sys


def _refresh_planner():
    """Refresh cached Rhino imports after an installed toolbox update."""
    import importlib
    import inspect
    from pathlib import Path

    import motion_toolbox.recording as recording
    if getattr(recording, 'RECORDING_VERSION', 0) < 3 and recording.current_run() is None:
        importlib.reload(recording)
    import motion_toolbox
    if getattr(motion_toolbox, '__version__', None) != '0.1.2':
        importlib.reload(motion_toolbox)

    names = (
        'motion_toolbox.kinematics.ur', 'motion_toolbox.kinematics.solver',
        'motion_toolbox.graph', 'motion_toolbox.planning',
        'motion_toolbox.robot_adapter', 'motion_toolbox.collision',
        'motion_toolbox.robot_planning',
    )
    modules = [importlib.import_module(name) for name in names]
    def stamp(module):
        path = Path(module.__file__).resolve()
        stat = path.stat()
        return str(path), stat.st_mtime_ns, stat.st_size

    parameters = inspect.signature(modules[-1].plan_robot).parameters
    stale = ('current_pose' not in parameters or
             parameters['current_pose'].default is inspect.Parameter.empty)
    stale = stale or getattr(modules[-1], 'ROBOT_COMPONENT_VERSION', 0) < 6
    stale = stale or any(
        getattr(module, '_robot_component_stamp', stamp(module)) != stamp(module)
        for module in modules)
    if stale:
        importlib.invalidate_caches()
        # Reload dependencies before their consumers so from-imports agree.
        for module in modules:
            importlib.reload(module)
    for module in modules:
        module._robot_component_stamp = stamp(module)


def plan(robot, targets, bases=None, current_pose=None, arm_in_base=None, arm_joint_names=None,
         collision_meshes=(), model_units_to_metres=1.0, **options):
    """Callable from standalone Python too; no Grasshopper imports required."""
    _refresh_planner()
    from motion_toolbox.robot_planning import plan_robot
    return plan_robot(robot, targets, bases, current_pose, arm_in_base, arm_joint_names,
                      collision_meshes, model_units_to_metres, **options)


def _input(name, default=None):
    value = globals().get(name)
    return default if value is None else value


joint_plan = None
configurations = []
base_result = []
path_cost = None
unreachable_points = []
result = None
status = ''
version = ''
timings = {}
diagnostics = []

try:
    source = _input('toolbox_src')
    if source and str(source) not in sys.path:
        sys.path.insert(0, str(source))
    result = plan(
        _input('robot'), list(_input('target_planes', [])), list(_input('base_planes', [])),
        _input('current_pose', []), _input('arm_in_base'), list(_input('arm_joint_names', [])),
        list(_input('collision_meshes', [])), _input('model_units_to_metres', 1.0),
        collision_check=_input('collision_check', True), check_edges=_input('check_edges', False),
        rotation_steps=_input('rotation_steps', 24), joint_ranges=_input('joint_ranges'),
        max_joint_step=_input('max_joint_step', 2.5), fixed_joint_values=_input('fixed_joint_values'),
        collision_options=_input('collision_options'), group=_input('group'),
        scene=_input('collision_scene'),
        parameters=_input('ur_parameters'),
    )
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path
    from motion_toolbox.geometry import to_rhino
    joint_plan = DataTree[float]()
    for i, q in enumerate(result['configurations']):
        for value in q:
            joint_plan.Add(float(value), GH_Path(i))
    configurations = result['configuration_objects']
    base_result = [to_rhino(p, 1.0/_input('model_units_to_metres', 1.0)) for p in result['base_planes']]
    path_cost = result['path_length']
    unreachable_points = result['unreachable_points']
    timings = result['timings']
    version = result['version']
    for i in unreachable_points:
        detail = result['target_diagnostics'][i]
        reason = ('no analytic IK' if not detail['raw_ik'] else
                  'joint limits' if not detail['within_joint_limits'] else 'collision rejection')
        diagnostics.append('Target {}: {}; raw IK={}, within limits={}, collision-free={}; {}'.format(
            i, reason, detail['raw_ik'], detail['within_joint_limits'], detail['collision_free'],
            detail['rejection_reasons']))
    if configurations:
        status = 'Planned {} targets; collision checking {}.'.format(len(configurations), 'on' if result['collision_check_applied'] else 'off')
    elif unreachable_points:
        status = 'No complete path. Targets without valid configurations after IK, limits and collisions: {}. See diagnostics.'.format(unreachable_points)
    else:
        status = 'No connected path satisfies joint-step / transition constraints.'
    for warning in result['warnings']:
        diagnostics.append(warning)
        status += ' Warning: ' + warning
        if globals().get('ghenv') is not None:
            from Grasshopper.Kernel import GH_RuntimeMessageLevel
            ghenv.Component.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, warning)
except Exception as error:
    joint_plan, configurations, base_result = None, [], []
    path_cost, result, unreachable_points = None, None, []
    timings, diagnostics = {}, []
    status = '{}: {}'.format(type(error).__name__, error)

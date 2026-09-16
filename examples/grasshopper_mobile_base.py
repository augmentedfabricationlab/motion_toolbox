"""Rhino 8 Python 3: plan base motion for printing while driving.

Load this file by path in a Grasshopper Python 3 component. Recomputes
when inputs change. Requires motion-toolbox with COMPAS FAB and PyBullet.
Mark optional inputs Optional. No external path generator is required.

Required inputs:
  robot              Item: COMPAS/MobileRobot, model and tool in metres
  target_planes      List, Plane: ordered world TCP planes

Optional inputs:
  seed_base_planes   List, Plane: optional proposals for strategy=discrete
  base_height        Item, float: footprint Z in model units (default 0)
  grid_spacing       Item, float: search spacing in model units (default 0.5 metre)
  yaw_steps          Item, int: heading samples at each arm origin (default 4)
  model_units_to_metres Item, float: 1 for metres, 0.001 for millimetres
  xy_tolerance       Item, float: ignored XY ripple size, model units (default 5 cm)
  mobile_options     Item, JSON: search/sampling/limits overrides (README)
                     Default strategy=smooth_offset: footprint +X faces wall,
                     lateral_distance=1.0 metres along footprint +/-Y;
                     wall_distances=[0.4,0.6,0.8,1.0,1.2] metres along +X;
                     smoothing_windows=[10,25,50,100,200], smooth_max_attempts=12.
                     connect_sections=true joins overlapping partial paths;
                     section_size=100, section_proposals=6, section_beam_width=2.
                     All original 3D targets and transitions are validated.
                     strategy=discrete enables previous grid/sparse settings.
  current_pose       List, float: optional six starting arm angles, radians
  start_base         Item, Plane: required with current_pose; model units
  arm_in_base        Item, Plane: optional calibrated mounting override
  arm_joint_names    List, str: optional six arm names in analytic IK order
  fixed_joint_values Item, JSON: fixed lift/non-arm joints, metres/radians
  collision_meshes   List, Mesh: environment geometry, model units
  collision_check    Item, bool: True by default
  check_edges        Item, bool: True; sampled full-body transition checks
  rotation_steps     Item, int: 1 by default (preserves TCP orientation)
  max_joint_step     Item, float: 2.5 radians per target by default
  joint_ranges       Item, JSON: optional arm limits; default model limits
  collision_options  Item, JSON: shared robot collision settings
  collision_scene    Item: optional preconfigured reusable scene
  group              Item, str: optional active tool/planning group
  ur_parameters      Item, JSON: optional UR geometry parameters
  toolbox_src        Item, str: optional development source path

Outputs:
  base_planes, base_result: footprint planes, exactly one per TCP on success
  joint_plan: DataTree of validated arm angles, branch {target_index}
  configurations: named lift/arm configurations
  path_cost: combined base/yaw/joint travel cost
  unreachable_points, result, status, timings, diagnostics, version

Connect base_planes to grasshopper.py's base_planes input if separate arm
planning is wanted; this component already returns a validated joint plan.
The calibrated arm origin must be behind target +Z and within 1.75 metres
in XY, using the stationary finder region at each target. These rules also
apply to every interpolated pose. Sparse search is approximate, interpolates
base poses only, validates every
original target/transition and falls back to dense search on failure. Limits
and speed timing are configured through mobile_options in metres/radians.
The planner assumes a holonomic upright base; it does not command motion or
produce acceleration/steering-constrained controller trajectories.
"""
import sys


def _refresh_planner():
    """Refresh cached Rhino imports after an installed toolbox update."""
    import importlib
    import inspect
    from pathlib import Path

    import motion_toolbox.recording as recording
    if getattr(recording, 'RECORDING_VERSION', 0) < 5 and recording.current_run() is None:
        importlib.reload(recording)
    import motion_toolbox
    if getattr(motion_toolbox, '__version__', None) != '0.1.12':
        importlib.reload(motion_toolbox)

    names = (
        'motion_toolbox.kinematics.ur', 'motion_toolbox.kinematics.solver',
        'motion_toolbox.graph', 'motion_toolbox.planning',
        'motion_toolbox.robot_adapter', 'motion_toolbox.collision',
        'motion_toolbox.mobile_transitions', 'motion_toolbox.base_planning', 'motion_toolbox.stationary_region', 'motion_toolbox.mobile_sections', 'motion_toolbox.smooth_mobile', 'motion_toolbox.mobile_planning',
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
    stale = stale or getattr(modules[-1], 'ROBOT_COMPONENT_VERSION', 0) < 16
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
    from motion_toolbox.geometry import as_plane
    from motion_toolbox.robot_planning import json_input
    settings = dict(sparse=True, strategy='smooth_offset')
    settings.update(json_input(options.pop('mobile_options', None), {}))
    tolerance = options.pop('xy_tolerance', .05/model_units_to_metres)*model_units_to_metres
    if settings['sparse']:
        settings.setdefault('xy_tolerance', tolerance)
    settings['placement_region'] = True
    settings['base_height'] = options.pop('base_height', 0.0)*model_units_to_metres
    settings['grid_spacing'] = options.pop('grid_spacing', .5/model_units_to_metres)*model_units_to_metres
    settings['yaw_steps'] = options.pop('yaw_steps', 4)
    start_base = options.pop('start_base', None)
    if start_base is not None:
        settings['start_base'] = as_plane(start_base, model_units_to_metres)
    options['mobile_options'] = settings
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
        _input('robot'), list(_input('target_planes', [])), list(_input('seed_base_planes', [])),
        _input('current_pose', []), _input('arm_in_base'), list(_input('arm_joint_names', [])),
        list(_input('collision_meshes', [])), _input('model_units_to_metres', 1.0),
        collision_check=_input('collision_check', True), check_edges=_input('check_edges', True),
        rotation_steps=_input('rotation_steps', 1), joint_ranges=_input('joint_ranges'),
        max_joint_step=_input('max_joint_step', 2.5), fixed_joint_values=_input('fixed_joint_values'),
        collision_options=_input('collision_options'), group=_input('group'),
        scene=_input('collision_scene'),
        parameters=_input('ur_parameters'),
        mobile_options=_input('mobile_options'),
        grid_spacing=_input('grid_spacing', .5/_input('model_units_to_metres', 1.0)),
        yaw_steps=_input('yaw_steps', 4),
        xy_tolerance=_input('xy_tolerance', .05/_input('model_units_to_metres', 1.0)),
        base_height=_input('base_height', 0.0), start_base=_input('start_base'),
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
    if result.get('unchecked_points'):
        diagnostics.append('{} targets were not checked in the reported attempt; see checked_in_any_attempt for overall coverage.'.format(len(result['unchecked_points'])))
    timings = result['timings']
    version = result['version']
    for i in unreachable_points:
        detail = result['target_diagnostics'][i]
        reason = ('placement/body rejection before IK' if not detail['raw_ik'] and detail['rejection_reasons'] else
                  'no analytic IK' if not detail['raw_ik'] else
                  'joint limits' if not detail['within_joint_limits'] else 'collision rejection')
        diagnostics.append('Target {}: {}; raw IK={}, within limits={}, collision-free={}; {}'.format(
            i, reason, detail['raw_ik'], detail['within_joint_limits'], detail['collision_free'],
            detail['rejection_reasons']))
    if configurations:
        status = 'Planned {} targets; collision checking {}.'.format(len(configurations), 'on' if result['collision_check_applied'] else 'off')
    elif unreachable_points:
        status = 'No complete path: {} targets have no feasible state. First indices: {}. See diagnostics for IK/limits/collision reasons.'.format(len(unreachable_points), unreachable_points[:12])
    else:
        status = 'No connected path satisfies joint-step / transition constraints.'
        blocked = next((d for d in result.get('mobile_diagnostics', [])
                        if d.get('reason') == 'transition_blocked'), None)
        if blocked:
            status = 'Blocked transition {} -> {} (zero-based): {}. See diagnostics for measured values and limits.'.format(
                blocked['from_target'], blocked['to_target'], blocked['rejection_counts'])
    if not configurations and any(d.get('reason') == 'smooth_proposals_exhausted' for d in result.get('mobile_diagnostics', [])):
        status = 'No validated smooth offset path in the tested proposal budget. See diagnostics for failed targets and transitions.'
    diagnostics.extend(str(item) for item in result.get('mobile_diagnostics', []))
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

# Primary output for the downstream arm-planning component.
base_planes = base_result

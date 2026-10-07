"""Rhino 8 Python 3: target planes -> one stationary robot footprint plane.

Evaluates automatically on every Grasshopper recompute.
Validated arm paths retain one shoulder/elbow/wrist branch, including current_pose
when supplied. Ambiguous branch boundaries are rejected, as in the mobile planner.

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
                   Missing active tool is an error, even with collision_check=False.

Optional inputs (defaults)
--------------------------
Arm and mounting:
  current_pose     List, float: six starting arm angles; empty excludes approach motion.
                   Or Item, no type hint: a named COMPAS Configuration, including lift.
                   Start joint limits and configuration collisions are checked.
  arm_in_base      Item, Plane: override; otherwise offline calibration or URDF controller base
  arm_joint_names List, str: optional override; inferred from robot/tool chain by default
  fixed_joint_values Item, JSON: nonplanned joints, e.g. {"lift": 0.2}, metres/radians
  ur_parameters   Item, JSON: optional six UR dimensions; default UR20
  group           Item, str: active tool group, if needed

Placement and units:
  candidate_planes List, Plane: optional exclusive footprint set. Heuristic mode
                   applies placement-region rules; adaptive mode uses IK/collision feasibility.
  units_to_metres  Item, float (1): supplied planes/meshes/search lengths use these units
  grid_spacing     Item, float (0.5 / units_to_metres): maximum grid spacing
  base_height      Item, float (0): footprint world Z
  yaw_steps        Item, int (4): evenly spaced footprint orientations
  max_validation_attempts Item, int (3): maximum final-position IK checks;
                   heuristic mode only; stop at first success

Arm validation and path:
  search_strategy Item, str ('heuristic'): 'adaptive' enables the adaptive base search.
  search_options  Item, JSON: optional adaptive settings; distances are metres.
                  tool_clearance defaults to 1 m as a footprint preference, never
                  a collision substitute. max_full_checks defaults to 3. The
                  search stops at the first connected finalist by default;
                  increase connected_finalists (1) to compare more paths. A sampled
                  search cannot prove a global optimum. fast_validation=False or
                  count_paths=True restores the existing exhaustive heuristic.
  rotation_steps   Item, int (24): TCP-Z orientation samples (1 fixes orientation)
  max_joint_step   Item, float (2.5): radians per arm joint per target step
  build_path       Item, bool (True): build a joint path; adaptive mode compares finalists
  count_paths      Item, bool (False): also count every possible joint path (slower)
  fast_validation Item, bool (True): skip unused collision alternatives;
                   False restores exact counts. count_paths=True overrides this.
  joint_ranges     Item, JSON: optional six [min,max] ranges in radians, null for unbounded.
  time_intervals   List, float or Item, JSON: positive seconds per transition;
                   N-1 values without current_pose, N with it (including approach).
  max_joint_speed  Item, number or JSON: positive rad/sec, scalar or six values;
                   requires time_intervals and limits steps together with max_joint_step.
  collision_meshes List, Mesh: wall and environment obstacles
                   (needed for full-body wall clearance)
  collision_check Item, bool (True): model/tool/environment at target configurations only
  collision_options Item, JSON: clearance/ground_z in metres; support_links,
                   allowed_pairs, check_static_self_collisions, exclude_gps,
                   excluded_collision_links, base_collision_model, package_paths,
                   asset_root, gui. Defaults retain detailed geometry and GPS.
  collision_scene  Item, no type hint: configured PybulletServer; remains open.
                   Must match robot/tool/environment and arm joint order. Omit
                   collision_meshes; only clearance may accompany a supplied scene.
  cancel           Item, bool (False): skip/cancel this run. A GH toggle does not
                   interrupt a synchronous run already in progress.
  cancel_check     Item, no type hint: optional callable polled during work;
                   return True or raise PlanningCancelled to cancel cooperatively.
  progress_callback Item, no type hint: optional callable receiving progress dicts.

Toolbox loading:
  toolbox_src     Item, str: motion_toolbox/src; restart Rhino when switching installations

Outputs
-------
Placement:
  base_plane: one Rhino footprint plane, None on failure.
  initial_base_plane: heuristic geometry guess, or highest-ranked adaptive probe;
                      available even if full arm validation fails.
  search_summary: adaptive search status, counters and validated candidates (JSON).
  search_diagnostics: adaptive proposal/probe diagnostics (JSON); sparse probe
                      counts are not exact all-target collision-free counts.
  placement_region: closed Curve in model units at calibrated arm-origin height;
                    boundary of the sampled XY reach/negative-side region for the
                    ARM ORIGIN, not the footprint. Before collision/IK checking;
                    conservative 128-sided reach disks. With candidate_planes,
                    shows the geometric envelope, not the exclusive candidate set.
                    Empty on errors/cancellation or when no polygon exists.
                    Adaptive search may extend outside this 1.75 m starting envelope.
  initial_guesses: geometry-valid Rhino footprint seeds;
                   IK/collisions not yet certified.
  standoff: minimum signed distance behind all target planes, metres.
  max_target_distance: farthest target from the calibrated arm-base origin,
                       metres.

Arm results:
  planned_tcp: List, Plane: selected world TCP waypoints in input model units,
               including selected TCP-Z rotations, in original target order.
               Empty without a complete joint path (also when build_path=False).
               Collision validation follows collision_check; transitions are unchecked.
  joint_plan: DataTree, one branch per target.
  path_cost: joint-path cost.
  path_count: exact integer text when count_paths=True; otherwise "not counted".
  solution_counts: exact feasible configurations per target; [] in fast mode.
  ik_option_count: product of exact counts, or "not counted" in fast mode.
  counts_complete: whether exhaustive configuration counts are available.
  configurations: List of named COMPAS Configurations (fixed joints plus arm).
  path_complete: complete connected joint path exists, regardless of collision toggle.
  valid: complete path AND configuration collision checks enabled. Transitions unchecked.
  cancelled: cooperative cancellation or KeyboardInterrupt cleared the outputs.

Diagnostics and timing:
  status, diagnostics: summary and detailed results.
  candidate_count: number of proposed bases.
  path_search_count: 0 or 1 for the returned base; adaptive totals are in search_summary.
  validation_attempts: number of expensive arm validations.
  base_collision_checks: number of base-only tests.
  elapsed_seconds: total component execution time.
  timings: seconds spent in geometry, collision setup, IK, joint expansion,
           collision checking and path search.
  disconnected_detail: JSON: zero-based transition indices (-1 means start),
            effective step limits and nearest candidate joint deltas/limit ratio.
  initial_state_failure: starting-pose collision reason, if candidate attempts fail.
  unreachable_points: zero-based failed target indices when no base succeeds.
  progress_messages: JSON progress messages, also printed to out.
  effective_settings: JSON of resolved units, calibration, joints and collision settings.
  loaded_code: JSON of source paths, package/API versions and loaded-code/file hashes.
  research_run: local recording directory, or None when recording is disabled.
  result: full workflow dictionary; None on error/cancellation.

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
import json
from decimal import Decimal
from time import perf_counter


def _input(name, default=None):
    value = globals().get(name)
    return default if value is None else value


base_plane, joint_plan, path_cost = None, None, None
planned_tcp = []
valid, path_complete, cancelled = False, False, False
configurations, progress_messages, unreachable_points = [], [], []
result, research_run, disconnected_detail, initial_state_failure = None, None, None, None
effective_settings, loaded_code = None, None
diagnostics, candidate_count, status = [], 0, ''
path_count, solution_counts = '0', []
ik_option_count, path_search_count = '0', 0
counts_complete = False
initial_guesses = []
placement_region = None
search_summary, search_diagnostics = None, []
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
    if (getattr(adapter, 'STATIONARY_ADAPTER_VERSION', 0) < 5 or
        not required.issubset(inspect.signature(adapter.kinematics_from_robot).parameters)):
        # Rhino retains Python modules between component recomputes. Refresh the
        # adapter only when its cached API is older than this component requires.
        importlib.invalidate_caches()
        adapter = importlib.reload(adapter)
    if (getattr(adapter, 'STATIONARY_ADAPTER_VERSION', 0) < 5 or
        not required.issubset(inspect.signature(adapter.kinematics_from_robot).parameters)):
        raise RuntimeError('Outdated toolbox file: {}. Set toolbox_src to the updated motion_toolbox/src directory and restart Rhino.'.format(
            toolbox_loaded_from))
    # Reload the planning dependency chain together: refreshing only the adapter
    # leaves old imported functions in base_planning/planning alive in Rhino.
    pipeline_names = (
        'motion_toolbox', 'motion_toolbox.execution',
        'motion_toolbox.kinematics.ur', 'motion_toolbox.kinematics.solver', 'motion_toolbox.kinematics.calibrated',
        'motion_toolbox.graph', 'motion_toolbox.configuration_branch', 'motion_toolbox.planning',
        'motion_toolbox.robot_adapter',
        'motion_toolbox.base_collision', 'motion_toolbox.collision',
        'motion_toolbox.base_planning', 'motion_toolbox.stationary_region', 'motion_toolbox.adaptive_stationary',
        'motion_toolbox.robot_planning', 'motion_toolbox.stationary_workflow',
    )
    # Inspect cached modules without importing new consumers first: a new
    # workflow can import APIs absent from a still-cached dependency.
    pipeline = [sys.modules.get(name) for name in pipeline_names]
    def source_stamp(module):
        path = Path(module.__file__).resolve()
        stat = path.stat()
        return str(path), stat.st_mtime_ns, stat.st_size
    base_module = pipeline[pipeline_names.index('motion_toolbox.base_planning')]
    graph_module = pipeline[pipeline_names.index('motion_toolbox.graph')]
    planner_arguments = {'objective', 'placement_region', 'build_path', 'base_collision', 'max_validation_attempts', 'fast_validation'}
    stale = any(module is None or getattr(module, '_stationary_loaded_stamp', None) != source_stamp(module)
                for module in pipeline)
    stale = stale or not callable(getattr(sys.modules.get('motion_toolbox.execution'), 'check_cancel', None))
    stale = stale or not planner_arguments.issubset(inspect.signature(base_module.find_stationary_base).parameters)
    stale = stale or 'path_count' not in getattr(graph_module.GraphResult, '__dataclass_fields__', {})
    result_fields = {'selected_target_planes', 'initial_state_failure', 'disconnected_detail',
                     'configuration_branch_check_applied', 'edge_rejection_reasons'}
    stale = stale or not result_fields.issubset(getattr(base_module.BasePlan, '__dataclass_fields__', {}))
    stale = stale or getattr(pipeline[-1], 'STATIONARY_WORKFLOW_VERSION', 0) < 3
    stale = stale or getattr(pipeline[pipeline_names.index('motion_toolbox.collision')], 'COLLISION_API_VERSION', 0) < 12
    if stale:
        importlib.invalidate_caches()
        for index, name in enumerate(pipeline_names):
            # Each dependency is current before importing/reloading its users.
            cached = sys.modules.get(name)
            module = importlib.import_module(name) if cached is None else importlib.reload(cached)
            module._stationary_loaded_stamp = source_stamp(module)
            pipeline[index] = module
        base_module = pipeline[pipeline_names.index('motion_toolbox.base_planning')]
        graph_module = pipeline[pipeline_names.index('motion_toolbox.graph')]
    if (not planner_arguments.issubset(inspect.signature(base_module.find_stationary_base).parameters)
        or 'path_count' not in getattr(graph_module.GraphResult, '__dataclass_fields__', {})
        or not result_fields.issubset(getattr(base_module.BasePlan, '__dataclass_fields__', {}))
        or getattr(pipeline[-1], 'STATIONARY_WORKFLOW_VERSION', 0) < 3):
        raise RuntimeError('Outdated planner at {}. Set toolbox_src to the updated motion_toolbox/src directory and restart Rhino.'.format(
            base_module.__file__))
    from motion_toolbox.geometry import to_rhino
    from motion_toolbox.stationary_workflow import plan_stationary_base
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path
    scale = _input('units_to_metres', 1.0)
    def _progress(message):
        global research_run
        research_run = message.get('research_run')
        text = json.dumps(message, sort_keys=True)
        progress_messages.append(text)
        print('Stationary planner: '+text)
        callback = _input('progress_callback')
        if callback is not None:
            callback(message)
    def _cancel():
        if _input('cancel', False):
            return True
        callback = _input('cancel_check')
        return callback() if callback is not None else False
    result = plan_stationary_base(_input('robot'), list(_input('target_planes', [])),
        candidate_planes=list(_input('candidate_planes', [])), units_to_metres=scale,
        current_pose=_input('current_pose'), arm_in_base=_input('arm_in_base'),
        arm_joint_names=_input('arm_joint_names'), group=_input('group'),
        fixed_joint_values=_input('fixed_joint_values'), parameters=_input('ur_parameters'),
        search_strategy=_input('search_strategy', 'heuristic'), search_options=_input('search_options'),
        grid_spacing=_input('grid_spacing'), base_height=_input('base_height', 0.),
        yaw_steps=_input('yaw_steps', 4), rotation_steps=_input('rotation_steps', 24),
        max_validation_attempts=_input('max_validation_attempts', 3),
        max_joint_step=_input('max_joint_step', 2.5), joint_ranges=_input('joint_ranges'),
        build_path=_input('build_path', True), count_paths=_input('count_paths', False),
        fast_validation=_input('fast_validation', True), collision_check=_input('collision_check', True),
        collision_meshes=list(_input('collision_meshes', [])), collision_options=_input('collision_options'),
        scene=_input('collision_scene'), time_intervals=_input('time_intervals'),
        max_joint_speed=_input('max_joint_speed'), cancel_check=_cancel, progress=_progress)
    found = result['found']
    targets = result['targets']
    if result['search_result'] is not None:
        adaptive_result = result['search_result']
        search_diagnostics = json.dumps(adaptive_result['search_diagnostics'])
        search_summary = json.dumps({k: v for k, v in adaptive_result.items() if k != 'search_diagnostics'})
    initial_guesses = [to_rhino(p, 1/scale) for p in result['initial_guesses']]
    if result['placement_region']:
        import Rhino.Geometry as rg
        placement_region = rg.PolylineCurve([
            rg.Point3d(*(float(v)/scale for v in point)) for point in result['placement_region']])
    candidate_count = result['candidate_count']
    timings = dict(result['timings'])
    valid, path_complete = result['valid'], result['path_complete']
    configurations = result['configuration_objects']
    research_run = result['research_run']
    initial_state_failure = result['initial_state_failure']
    unreachable_points = result['unreachable_points']
    disconnected_detail = json.dumps(result['disconnected_detail'], sort_keys=True) if result['disconnected_detail'] else None
    effective_settings = json.dumps(result['effective_settings'], sort_keys=True)
    loaded_code = json.dumps(result['loaded_modules'], sort_keys=True)
    path_cost = found.cost if found.path_search_count else None
    planned_tcp = [to_rhino(p, 1/scale) for p in found.selected_target_planes]
    validation_attempts, base_collision_checks = found.validation_attempts, found.base_collision_checks
    initial_base_plane = to_rhino(found.heuristic_plane, 1/scale) if found.heuristic_plane is not None else None
    counts_complete = found.counts_complete
    path_search_count = found.path_search_count
    ik_option_count = str(Decimal(found.ik_option_count)) if counts_complete else 'not counted'
    diagnostics = ['Base {}: {}; IK checked {}; reachable {} of {} checked targets ({} total); solutions {}; path checked {}; '
        'standoff {:.3f} m; farthest 3D {:.3f} m; wrong-side targets {}; too-far XY targets {}'.format(
        i, d['reason'], d['ik_checked'], d['reachable_targets'], d['targets_checked'], len(targets),
        ('not counted' if d.get('counts_complete') is False else
         ('{}-{} per target'.format(min(d['solution_counts']), max(d['solution_counts'])) if d['solution_counts'] else 'none')),
        d['path_checked'], d['standoff'], d['max_target_distance'],
        d['wrong_side_points'], d['too_far_points'])
        for i, d in enumerate(found.diagnostics)]
    diagnostics.insert(0, 'Mounting: {}; arm origin in footprint {} m; fixed joints {}'.format(
        result['mounting_source'], result['arm_in_base'].origin.tolist(), result['fixed_joint_values']))
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
    diagnostics.extend(['Effective settings: '+effective_settings, 'Research run: '+str(research_run),
                        'CPU execution: '+json.dumps(result.get('execution_policy', {}), sort_keys=True)])
    if disconnected_detail:
        diagnostics.append('Disconnected path: '+disconnected_detail)
    if initial_state_failure:
        diagnostics.append('Starting configuration: '+initial_state_failure)
    if not counts_complete:
        diagnostics.append('Fast validation: alternative configuration counts not counted. Set fast_validation=False for exact counts.')
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
    status = result['status']
except (Exception, KeyboardInterrupt) as error:
    cancelled = type(error).__name__ in ('PlanningCancelled', 'KeyboardInterrupt')
    valid, path_complete = False, False
    configurations, unreachable_points = [], []
    result, disconnected_detail, initial_state_failure = None, None, None
    effective_settings, loaded_code = None, None
    initial_guesses, initial_base_plane = [], None
    placement_region = None
    search_summary, search_diagnostics = None, []
    planned_tcp = []
    counts_complete = False
    base_plane, joint_plan, path_cost = None, None, None
    path_count, solution_counts = '0', []
    ik_option_count, path_search_count = '0', 0
    standoff, max_target_distance = None, None
    status = 'Planning cancelled; no trajectory returned.' if cancelled else '{}: {}'.format(type(error).__name__, error)
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

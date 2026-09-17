"""Rhino 8 Python 3: smooth base proposal, then calibrated IK/collision validation.

Load this file by path and recompute. Required: robot (Item), target_planes (List).
Robot, active tool, calibration and fixed joints use metres/radians.
Optional inputs:
  model_units_to_metres / units_to_metres: default 1 (metres), matching captures.
    Set 0.001 explicitly for millimetre-valued input geometry.
  max_xy_deviation: model units, default 0.25 m converted to model units.
  normal_offset, tangent_offset: model units, defaults 0.9 m and 1.2 m.
  current_pose: optional six arm radians or named Configuration at first base.
  arm_in_base: calibrated controller-base plane in model units; normally inferred.
  arm_joint_names, fixed_joint_values, joint_ranges, group, ur_parameters.
  collision_meshes: environment meshes, model units.
  collision_options: JSON; allowed pairs, ground, sampling resolutions, etc.
  collision_scene: optional fully configured external scene.
  rotation_steps: default 16 equally spaced orientations about local TCP Z.
  max_joint_step: default 2.5 rad; max_base_step: default 0.25 m in model units;
  max_yaw_step: default 0.25 rad.
  time_intervals: optional seconds, N-1 transitions (N with current_pose).
  max_base_speed: model units/sec; max_yaw_speed, max_joint_speed: rad/sec.
  toolbox_src: optional source override.
Outputs:
  base_planes / base_result: one geometric proposal per original TCP, even on
    validation failure. Never interpret these alone as a validated plan.
  configurations / joint_plan: ONLY a complete validated connected arm trajectory.
  valid, status, diagnostics, result, timings, unreachable_points, target_indices.
  base_path, averaged_line, centerline: geometry previews.
Collision and sampled swept checks are mandatory. TCP positions/Z axes stay fixed.
selected_target_planes and selected_tcp_rotations report the validated selection.
Planning has no runtime limit. No robot is commanded.
"""
import importlib
import json
import math
import sys
import time
from pathlib import Path


def _input(name, default=None):
    value=globals().get(name)
    return default if value is None else value


base_planes,base_result,configurations,target_indices=[],[],[],[]
base_path,averaged_line,centerline,joint_plan=None,None,None,None
result,path_cost=None,None
valid=False
selected_target_planes,selected_tcp_rotations=[],[]
status=''
diagnostics,timings,unreachable_points=[],{},[]
version='0.1.30'
started=time.monotonic()

try:
    source=_input('toolbox_src',str(Path(__file__).resolve().parents[1]/'src'))
    if str(source) in sys.path:sys.path.remove(str(source))
    sys.path.insert(0,str(source))
    importlib.invalidate_caches()
    import motion_toolbox
    importlib.reload(motion_toolbox)
    # Rhino keeps modules alive between recomputes. Refresh dependencies before
    # consumers, including their from-import bindings to older solver functions.
    for module_name in ('kinematics.ur','kinematics.solver','kinematics.calibrated',
                        'graph','planning','robot_adapter','collision','robot_planning',
                        'xy_averaging','xy_smoothing','xy_centerline','xy_offset','mobile_base_workflow'):
        importlib.reload(importlib.import_module('motion_toolbox.'+module_name))
    from motion_toolbox.mobile_base_workflow import plan_mobile_base
    from motion_toolbox.robot_planning import json_input
    from motion_toolbox.geometry import to_rhino
    import Rhino
    import Rhino.Geometry as rg
    scale=_input('model_units_to_metres',_input('units_to_metres',1.))
    scale=float(scale)
    if not math.isfinite(scale) or scale<=0:raise ValueError('Positive finite model_units_to_metres required')
    # Do not silently honor stale switches that disable requested validation.
    if not _input('collision_check',True) or not _input('check_edges',True):
        raise ValueError('Mobile validation requires collision_check and check_edges enabled')
    result=plan_mobile_base(_input('robot'),list(_input('target_planes',[])),units_to_metres=scale,
        max_xy_deviation=float(_input('max_xy_deviation',.25/scale))*scale,
        normal_offset=float(_input('normal_offset',.9/scale))*scale,
        tangent_offset=float(_input('tangent_offset',1.2/scale))*scale,
        current_pose=_input('current_pose'),arm_in_base=_input('arm_in_base'),
        arm_joint_names=_input('arm_joint_names'),fixed_joint_values=_input('fixed_joint_values'),
        joint_ranges=_input('joint_ranges'),group=_input('group'),parameters=_input('ur_parameters'),
        collision_meshes=list(_input('collision_meshes',[])),collision_options=_input('collision_options'),
        scene=_input('collision_scene'),rotation_steps=_input('rotation_steps',16),max_joint_step=_input('max_joint_step',2.5),
        max_base_step=float(_input('max_base_step',.25/scale))*scale,max_yaw_step=_input('max_yaw_step',.25),
        time_intervals=json_input(_input('time_intervals')),
        max_base_speed=None if _input('max_base_speed') is None else float(_input('max_base_speed'))*scale,
        max_yaw_speed=_input('max_yaw_speed'),max_joint_speed=json_input(_input('max_joint_speed')))
    base_planes=[to_rhino(b,1./scale) for b in result['base_planes']]
    base_result=base_planes
    def line(points):return rg.PolylineCurve([rg.Point3d(float(p[0])/scale,float(p[1])/scale,0.) for p in points])
    base_path=line([b.origin for b in result['base_planes']])
    averaged_line=line(result['smoothing']['curve'])
    centerline=line(result['centerline']['curve'])
    target_indices=result['target_indices']
    valid=result['fabrication_validated']
    configurations=result['configuration_objects']
    selected_target_planes=[to_rhino(p,1./scale) for p in result['selected_target_planes']]
    selected_tcp_rotations=result['selected_tcp_rotations']
    path_cost=result['path_length']
    status=result['status']
    unreachable_points=result['unreachable_points']
    # GH panels display raw Python dictionaries as a .NET type name.
    result['effective_settings'].update(component_version=version,
        planner_path=str(importlib.import_module('motion_toolbox.mobile_base_workflow').__file__))
    diagnostics=[json.dumps(result['effective_settings'],sort_keys=True)]
    diagnostics += [json.dumps(d,sort_keys=True) for d in result['target_diagnostics'] if d['state']!='feasible_state']
    diagnostics += [json.dumps(d,sort_keys=True) for d in result['transition_failures']]
    if result['disconnected_target'] is not None:
        diagnostics.append(json.dumps(result['disconnected_detail'],sort_keys=True))
    if not result['smoothing']['converged']:
        diagnostics.append('Smoothing reached its iteration cap; deviation bound holds but objective convergence is unconfirmed.')
    timings=dict(validation_seconds=result['validation_seconds'],component_seconds=time.monotonic()-started)
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path
    joint_plan=DataTree[float]()
    for i,q in enumerate(result['configurations']):
        for value in q:joint_plan.Add(float(value),GH_Path(i))
except Exception as error:
    selected_target_planes,selected_tcp_rotations=[],[]
    valid=False
    base_planes,base_result,configurations,target_indices=[],[],[],[]
    base_path,averaged_line,centerline,joint_plan=None,None,None,None
    result,path_cost=None,None
    status='{}: {}'.format(type(error).__name__,error)
    diagnostics=[status]
    timings=dict(component_seconds=time.monotonic()-started)

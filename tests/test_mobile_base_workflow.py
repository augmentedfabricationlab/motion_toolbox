import runpy
import sys
from pathlib import Path
from types import ModuleType
import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.mobile_base_workflow import validate_base_path
from test_grasshopper_entrypoints import gh, robot_fixture


class Solver:
    arm_in_base = Plane.world_xy()
    revolute_joints = ()
    def __call__(self,target,base):
        return [[float(target.origin[0])]*6]


class World:
    last_failure = None
    def is_base_valid(self,base,**kw): return True
    def is_valid(self,q,base,**kw): return True
    def edge_is_valid(self,q0,b0,q1,b1,**kw): return True


def fixture():
    targets=[Plane((x,0,1),(0,0,1),(1,0,0)) for x in [0.,.1,.2]]
    bases=[Plane((x,-1,0),(0,1,0),(-1,0,0)) for x in [0.,.1,.2]]
    return targets,bases


def validate(world=None,solver=None,**kw):
    return validate_base_path(*fixture(),world=world or World(),solver=solver or Solver(),
        joint_ranges=[[-3,3]]*6,periodic=[False]*6,**kw)


def test_complete_path_requires_every_target_and_swept_join():
    world=World()
    checked=[]
    world.edge_is_valid=lambda q0,b0,q1,b1,**kw: checked.append((q0,q1)) or True
    result=validate(world)
    assert result['fabrication_validated']
    assert len(result['configurations'])==3 and len(checked)==2


def test_rotation_search_recovers_path_and_reports_selected_orientations():
    class RotatedOnly(Solver):
        def __call__(self,target,base):
            # Fixture TCP X initially points up. A local-Z quarter turn makes it +X.
            return super().__call__(target,base) if target.xaxis[0]>.999 else []
    original=validate(solver=RotatedOnly(),rotation_steps=1)
    assert original['state_counts']=={'no_ik':3}
    rotated=validate(solver=RotatedOnly(),rotation_steps=16)
    assert rotated['fabrication_validated']
    np.testing.assert_allclose(rotated['selected_tcp_rotations'],np.pi/2)
    for before,after in zip(fixture()[0],rotated['selected_target_planes']):
        np.testing.assert_allclose(before.origin,after.origin)
        np.testing.assert_allclose(before.zaxis,after.zaxis,atol=1e-12)


def test_rotation_search_does_not_add_revolutions_to_reach_start():
    class RotatedOnly(Solver):
        revolute_joints=tuple(range(6))
        def __call__(self,target,base):
            return [[-.4]*6] if target.xaxis[0]>.999 else []
    result=validate_base_path(*fixture(),world=World(),solver=RotatedOnly(),
        joint_ranges=[[-2*np.pi,2*np.pi]]*6,periodic=[False]*6,
        current_pose=[2*np.pi-.4]*6,max_joint_step=.1,rotation_steps=16)
    assert not result['fabrication_validated']
    assert result['disconnected_target']==0
    assert all(d['collision_free']==1 for d in result['target_diagnostics'])
    assert result['selected_tcp_rotations']==[]


def test_collision_and_no_ik_are_separate_and_all_targets_tested():
    world=World()
    world.last_failure='tool collision: tool[0] / forearm'
    world.is_valid=lambda q,b,**kw:q[0]!=0.
    class Missing(Solver):
        def __call__(self,t,b):return [] if t.origin[0]==.1 else super().__call__(t,b)
    result=validate(world,Missing())
    assert [d['state'] for d in result['target_diagnostics']]==['configuration_collision','no_ik','feasible_state']
    assert not result['fabrication_validated'] and not result['configurations']
    assert result['unchecked_points']==[]


def test_equivalent_sweep_cache_preserves_path_and_distinct_windings():
    class Turns(Solver):
        revolute_joints=(0,)
        def __call__(self,target,base):
            return [[.2+target.origin[0]+turn,0,0,0,0,0] for turn in (0,-2*np.pi)]
    class Counted(World):
        def __init__(self,cached):
            self.calls=0
            if not cached: self.configuration_cache_key=None
        def configuration_cache_key(self,q):
            return tuple(np.round([(q[0]+np.pi)%(2*np.pi)-np.pi]+list(q[1:]),10))
        def edge_is_valid(self,q0,b0,q1,b1,**kw):
            self.calls+=1
            # Endpoints differing by a full revolution do not imply the same sweep.
            return abs(q1[0]-q0[0])<1
    results=[];worlds=[]
    for cached in (False,True):
        world=Counted(cached);worlds.append(world)
        results.append(validate_base_path(*fixture(),world=world,solver=Turns(),
            joint_ranges=[[-2*np.pi,2*np.pi]]+[[-3,3]]*5,periodic=[False]*6,
            max_joint_step=7,rotation_steps=1))
    assert results[0]['fabrication_validated'] and results[1]['fabrication_validated']
    np.testing.assert_allclose(results[0]['configurations'],results[1]['configurations'])
    assert results[0]['edge_rejection_reasons']==results[1]['edge_rejection_reasons']
    assert results[1]['edge_cache_hits']>0
    assert worlds[1].calls<worlds[0].calls


def test_disconnected_transition_has_exact_target_and_collision_reason():
    world=World()
    world.last_failure='tool collision: tool[0] / wall'
    world.edge_is_valid=lambda *args,**kw:False
    result=validate(world)
    assert result['disconnected_target']==1
    assert 'tool collision' in str(result['edge_rejection_reasons'])
    assert not result['configurations']


def test_base_step_and_speed_limits_are_not_bypassed():
    result=validate(max_base_step=.05)
    assert result['transition_failures'][0]['rejections'][0]['measured']==pytest.approx(.1)
    assert not result['fabrication_validated']
    result=validate(time_intervals=[1,1],max_joint_speed=.05)
    assert result['disconnected_target']==1
    with pytest.raises(ValueError,match='require time_intervals'):
        validate(max_base_speed=.1)


def test_placement_rejection_is_not_mislabeled_as_no_ik():
    targets,bases=fixture()
    bases[1]=Plane((3,-1,0),(0,1,0),(-1,0,0))
    result=validate_base_path(targets,bases,solver=Solver(),world=World(),joint_ranges=[[-3,3]]*6,periodic=[False]*6)
    d=result['target_diagnostics'][1]
    assert d['state']=='placement_rejection' and d['raw_ik'] is None
    assert d['placement']['max_projected_distance']>1.75


def test_actual_component_returns_proposal_on_real_robot_ik_failure(gh,monkeypatch):
    rhino,geometry=ModuleType('Rhino'),ModuleType('Rhino.Geometry')
    geometry.Point3d=lambda *p:p
    geometry.PolylineCurve=lambda p:p
    rhino.Geometry=geometry
    # Metre-valued GH geometry must not be silently rescaled by document units.
    rhino.RhinoDoc=type('Doc',(),{'ActiveDoc':type('Active',(),{'ModelUnitSystem':'Millimeters'})()})
    rhino.UnitSystem=type('Units',(),{'Meters':'Meters'})
    rhino.RhinoMath=type('Math',(),{'UnitScale':staticmethod(lambda *args:.001)})
    monkeypatch.setitem(sys.modules,'Rhino',rhino)
    monkeypatch.setitem(sys.modules,'Rhino.Geometry',geometry)
    from compas.geometry import Frame
    robot,_,_,names=robot_fixture()
    robot._RCF=Frame.worldXY()
    targets=[Plane((x,0,10),(0,0,1),(1,0,0)) for x in [0.,.1,.2]]
    path=Path(__file__).resolve().parents[1]/'examples/grasshopper_mobile_base.py'
    import motion_toolbox.kinematics.ur as ur
    import motion_toolbox.kinematics.solver as solver_module
    stale=lambda *args:[]
    monkeypatch.setattr(ur,'inverse_kinematics',stale)
    monkeypatch.setattr(solver_module,'inverse_kinematics',stale)
    out=runpy.run_path(str(path),init_globals=dict(robot=robot,target_planes=targets,
        arm_joint_names=names,max_xy_deviation=.02))
    assert len(out['base_planes'])==3,out['status']
    assert out['target_indices']==[0,1,2]
    assert not out['valid'] and not out['configurations']
    assert out['result']['state_counts']['no_ik']==3
    assert ur.inverse_kinematics is not stale
    assert solver_module.inverse_kinematics is ur.inverse_kinematics
    assert all(isinstance(d,str) for d in out['diagnostics'])
    import json
    settings=json.loads(out['diagnostics'][0])
    assert settings['units_to_metres']==1
    assert settings['rotation_steps']==16
    assert settings['first_target']['origin'][2]==10

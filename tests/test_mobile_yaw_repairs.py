import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox import mobile_base_workflow as workflow
from test_mobile_base_workflow import Solver, World


@pytest.mark.parametrize('sign',[-1,1])
@pytest.mark.parametrize('slider',[17.,170.])
def test_rotation_repairs_base_collision_with_fixed_origin_and_full_checks(monkeypatch,sign,slider):
    targets=[Plane((i*.02,0,1),(0,0,1),(1,0,0)) for i in range(33)]
    bases=[Plane((i*.02,-1,0),(0,1,0),(-1,0,0)) for i in range(33)]
    monkeypatch.setattr(workflow,'generate_base_path',lambda *a,**k:dict(
        base_planes=bases,target_indices=list(range(33)),smoothing={},centerline={}))
    def adjustment(b):
        return (np.degrees(np.arctan2(b.xaxis[1],b.xaxis[0]))-90-slider+180)%360-180
    world=World();checked=[]
    def base_valid(b,**kwargs):
        # The middle target requires a rotation; no translation can solve it.
        i=round(b.origin[0]/.02)
        return (abs(b.origin[1]+1)<1e-12 and abs(b.origin[0]-i*.02)<1e-12
                and (i!=16 or sign*adjustment(b)>=12.5-1e-8))
    def full_valid(q,b,**kwargs):
        checked.append(b)
        # A direction that clears the base can still collide with the arm/tool.
        return sign*adjustment(b)<=22.5+1e-8
    world.is_base_valid=base_valid;world.is_valid=full_valid
    kw=dict(solver=Solver(),world=world,joint_ranges=[[-3,3]]*6,periodic=[False]*6,
            rotation_steps=1,base_yaw_degrees=slider,max_yaw_step=np.radians(2))
    result=workflow.plan_base_path(targets,**kw)
    assert result['fabrication_validated'],result['status']
    np.testing.assert_allclose([b.origin for b in result['base_planes']],[b.origin for b in bases],atol=1e-12)
    angles=result['applied_yaw_adjustments_degrees']
    assert 12.5-1e-8<=sign*angles[16]<=22.5+1e-8
    assert abs(angles).max()<=30+1e-8
    assert abs(np.diff(angles)).max()<=2+1e-8
    assert checked and len(result['configurations'])==33
    np.testing.assert_allclose([adjustment(b) for b in result['base_planes']],angles,atol=1e-10)
    np.testing.assert_allclose(result['applied_base_yaw_degrees'],slider+angles)
    assert result['applied_offsets'].shape==(33,2)


@pytest.mark.parametrize('margin',[-1,181,float('nan'),float('inf')])
def test_invalid_yaw_margin_fails_before_planning(margin):
    with pytest.raises(ValueError,match='base_yaw_margin_degrees'):
        workflow.plan_base_path([],solver=None,world=None,joint_ranges=[],periodic=[],base_yaw_margin_degrees=margin)


def test_valid_path_does_not_use_yaw_allowance():
    class Zero(Solver):
        def __call__(self,t,b):return [[0.]*6]
    targets=[Plane((i*.03,0,1),(0,0,1),(1,0,0)) for i in range(20)]
    kw=dict(solver=Zero(),world=World(),joint_ranges=[[-3,3]]*6,periodic=[False]*6,rotation_steps=1)
    a=workflow.plan_base_path(targets,**kw)
    b=workflow.plan_base_path(targets,base_yaw_margin_degrees=0,**kw)
    assert a['fabrication_validated'] and b['fabrication_validated']
    np.testing.assert_array_equal([p.matrix for p in a['base_planes']],[p.matrix for p in b['base_planes']])
    np.testing.assert_array_equal(a['applied_yaw_adjustments_degrees'],0)


@pytest.mark.parametrize('tangent',[-.3,.3])
def test_combined_arc_offset_and_yaw_repair_respects_custom_margin(tangent):
    from test_xy_sections import arc,prepared,targets as tcp_targets
    from motion_toolbox.mobile_adaptation import repair_offsets
    points,normals=arc(count=33,center=(0,0))
    geometry=prepared(points,normals)
    bases=geometry.frames([.5,tangent])
    def angle(b):
        inward=-b.origin[:2]/np.linalg.norm(b.origin[:2])
        return np.degrees(np.arctan2(inward[0]*b.xaxis[1]-inward[1]*b.xaxis[0],np.dot(inward,b.xaxis[:2])))
    def feasible(i,b):
        return i!=16 or (np.linalg.norm(b.origin[:2])>=2.6-1e-8 and angle(b)>=17.3-1e-8)
    def validate(bs):
        failed=[i for i,b in enumerate(bs) if not feasible(i,b)]
        return dict(fabrication_validated=not failed,unreachable_points=failed,
                    transition_failures=[],disconnected_target=None,status='test',base_planes=bs)
    kw=dict(validate=validate,probe=lambda ids,bs:all(feasible(i,b) for i,b in zip(ids,bs)),
            normal_offset=.5,tangent_offset=tangent,search_extent=.8,build_frames=geometry.frames)
    ts=tcp_targets(points,normals)
    blocked,_,_=repair_offsets(ts,bases,validate(bases),yaw_margin_degrees=10,**kw)
    assert not blocked['fabrication_validated']
    solved,applied,attempts=repair_offsets(ts,bases,validate(bases),yaw_margin_degrees=17.3,**kw)
    assert solved['fabrication_validated']
    assert abs(np.degrees(applied[:,2])).max()<=17.3+1e-8
    assert applied[16,0]>=.6-1e-8
    for i,b in enumerate(solved['base_planes']):
        assert angle(b)==pytest.approx(np.degrees(applied[i,2]),abs=1e-8)
        assert np.linalg.norm(b.origin[:2])==pytest.approx(2+applied[i,0],abs=1e-8)
    assert attempts[-1]['yaw_adjustment_degrees']==pytest.approx(17.3)

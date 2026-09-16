import numpy as np
from motion_toolbox.adaptive_mobile import plan_adaptive_mobile
from motion_toolbox.geometry import Plane


def wall(x):
    return Plane((x,0,1),(1,0,0),(0,0,-1))


def test_failure_between_knots_refines_controls_and_validates_every_target():
    targets = [wall(i*.01) for i in range(11)]
    def placement(target,base):
        return abs(target.origin[0]-.05)>1e-8 or base.origin[1]<-.9
    edges = []
    result = plan_adaptive_mobile(targets,Plane.world_xy(),window=3,knot_gap=10,
        wall_distances=[.8,1.],lateral_offsets=[0],yaw_offsets=[0],
        ik_solver=lambda t,b:[[0.]],base_valid=placement,
        transition_check=lambda q0,b0,q1,b1: edges.append((b0,b1)) or True)
    assert len(result.configurations)==len(result.base_planes)==len(targets)
    assert all(placement(t,b) for t,b in zip(targets,result.base_planes))
    assert result.diagnostics[-1]['iterations'][0]['failed_targets']==[5]
    assert len(edges)>=10
    assert np.max(np.linalg.norm(np.diff([b.origin for b in result.base_planes],axis=0),axis=1))<.25


def test_rejects_disconnected_trajectory_without_returning_partial_output():
    result = plan_adaptive_mobile([wall(i*.01) for i in range(11)],Plane.world_xy(),
        window=3,knot_gap=10,rounds=2,wall_distances=[1.],lateral_offsets=[0],yaw_offsets=[0],
        ik_solver=lambda t,b:[[0.]],transition_check=lambda *args:False)
    assert not result.configurations
    assert not result.base_planes
    assert any(d.get('reason')=='transition_blocked' for d in result.diagnostics)

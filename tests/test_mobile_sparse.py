import math
import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.base_planning import plan_mobile_base
from motion_toolbox.mobile_planning import significant_targets, interpolate_bases, plan_mobile_sparse


def plane(x, y=0):
    return Plane((x,y,0), (1,0,0), (0,1,0))


def test_keyframes_turns_rotation_duplicates_and_gaps():
    frames = [plane(0), plane(.01), plane(.02), plane(.02,.01), plane(.02,.01).rotated_z(.4)]
    assert significant_targets(frames, max_gap=20) == [0,2,4]
    assert significant_targets([plane(i*.001) for i in range(10)], max_gap=3) == [0,3,6,9]
    with pytest.raises(ValueError):
        significant_targets(frames, max_gap=0)


def test_interpolation_short_yaw_arc_and_nonuniform_coordinates():
    a, b = plane(0).rotated_z(math.radians(179)), plane(1).rotated_z(math.radians(-179))
    result = interpolate_bases([0,2], [a,b], [0,.25,1])
    assert result[1].origin[0] == .25
    assert math.degrees(math.atan2(result[1].xaxis[1], result[1].xaxis[0])) == pytest.approx(179.5)


def test_sparse_reduces_ik_and_validates_every_original_target():
    targets = [plane(i*.01) for i in range(101)]
    calls = []
    def ik(t,b):
        calls.append(t.origin[0])
        return [[float(t.origin[0]-b.origin[0])]]
    layers = [[plane(i*.01,j*.01) for j in range(8)] for i in range(101)]
    dense = plan_mobile_base(targets,layers,ik_solver=ik)
    dense_calls = len(calls)
    calls.clear()
    sparse = plan_mobile_sparse(targets,layers,ik_solver=ik)
    assert len(sparse.base_planes) == len(sparse.configurations) == 101
    assert set(calls) == {t.origin[0] for t in targets}
    assert len(calls) < dense_calls/3
    assert sparse.cost == pytest.approx(dense.cost)
    assert not sparse.diagnostics[-1]['dense_fallback']


def test_skipped_collision_and_transition_force_dense_fallback():
    targets = [plane(i*.01) for i in range(5)]
    layers = [[plane(i*.01),plane(i*.01,.1)] for i in range(5)]
    def collision(q,b):
        return not (abs(b.origin[0]-.02)<1e-9 and abs(b.origin[1])<1e-9)
    result = plan_mobile_sparse(targets,layers,ik_solver=lambda t,b:[[0]],collision=collision)
    assert result.diagnostics[-1]['dense_fallback']
    assert result.base_planes[2].origin[1] == .1
    result = plan_mobile_sparse(targets,layers,ik_solver=lambda t,b:[[0]], transition_check=lambda *args:False)
    assert not result.base_planes
    assert result.diagnostics[-1]['dense_fallback']


def test_time_interpolation_and_speed_validation():
    targets = [plane(0),plane(.01),plane(.02)]
    result = plan_mobile_sparse(targets,[[plane(0)],[plane(.1)],[plane(.2)]],
        ik_solver=lambda t,b:[[0]], time_intervals=[1,3],max_base_speed=.06)
    assert result.base_planes[1].origin[0] == pytest.approx(.05)
    rejected = plan_mobile_sparse(targets,[[plane(0)],[plane(.1)],[plane(.2)]],
        ik_solver=lambda t,b:[[0]], time_intervals=[1,3],max_base_speed=.01)
    assert not rejected.base_planes


def test_approach_time_and_single_target():
    result = plan_mobile_sparse([plane(.01)],[[plane(.01)]],ik_solver=lambda t,b:[[0]],
        current_pose=[0],start_base=plane(0),time_intervals=[1],max_base_speed=.02)
    assert len(result.base_planes) == 1
    assert not plan_mobile_sparse([plane(.01)],[[plane(.01)]],ik_solver=lambda t,b:[[0]],
        current_pose=[0],start_base=plane(0),time_intervals=[1],max_base_speed=.001).base_planes


def test_mobile_json_cannot_disable_robot_collision_checks():
    from motion_toolbox.mobile_planning import plan_mobile_robot_path
    with pytest.raises(ValueError, match='Unknown mobile options'):
        plan_mobile_robot_path([plane(0)],[plane(0)],{'collision':None},
                               ik_solver=lambda t,b:[[0]],collision=lambda q,b:False)

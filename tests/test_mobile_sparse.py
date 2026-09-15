import math
import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.base_planning import plan_mobile_base
from motion_toolbox.mobile_planning import significant_targets, interpolate_bases, plan_mobile_sparse
from motion_toolbox.mobile_planning import xy_feature_targets, xy_feature_progress


def square_with_ripple():
    vertices = np.array([[0,0],[1,0],[1,1],[2,1],[2,0],[3,0]], dtype=float)
    points = []
    for a,b in zip(vertices, vertices[1:]):
        chord = b-a
        normal = np.array([-chord[1],chord[0]])
        for f in np.linspace(0,1,200,endpoint=False):
            points.append(a+f*chord+.01*np.sin(f*20*np.pi)*normal)
    points.append(vertices[-1])
    return [plane(*p) for p in points]


def test_xy_features_ignore_ripples_but_keep_square_corners():
    targets = square_with_ripple()
    kept = xy_feature_targets(targets,xy_tolerance=.05)
    assert len(kept) == 6
    assert all(abs(a-b) <= 5 for a,b in zip(kept,[0,200,400,600,800,1000]))
    points = np.array([t.origin[:2] for t in targets])
    for a,b in zip(kept,kept[1:]):
        chord = points[b]-points[a]
        delta = points[a:b+1]-points[a]
        f = np.clip(delta @ chord/(chord @ chord),0,1)
        assert np.linalg.norm(delta-f[:,None]*chord,axis=1).max() <= .05
    progress = xy_feature_progress(targets,kept)
    assert np.all(np.diff(progress) >= 0)
    assert progress[-1] == pytest.approx(5.,abs=.05)  # Uses simplified path length.


def test_xy_features_ignore_height_and_tcp_roll_by_default():
    targets = [Plane((0,0,i*.01),(1,0,0),(0,1,0)).rotated_z(i*.2) for i in range(101)]
    assert xy_feature_targets(targets) == [0,100]
    assert xy_feature_targets(targets,z_tolerance=.4) == [0,40,80,100]


def test_square_wave_plans_keyframes_but_returns_and_checks_every_target():
    targets = square_with_ripple()
    seen = set()
    layers_requested = []
    def layer(i,t):
        layers_requested.append(i)
        return [plane(t.origin[0],t.origin[1]+j*.1) for j in range(4)]
    def collision(q,b):
        seen.add(round(q[0],8))
        return True
    indices = {tuple(t.origin):i for i,t in enumerate(targets)}
    result = plan_mobile_sparse(targets,layer,ik_solver=lambda t,b:[[indices[tuple(t.origin)]*.001]],
                                collision=collision)
    assert len(result.base_planes) == len(result.configurations) == 1001
    assert len(layers_requested) == 6
    assert len(seen) == 1001
    assert result.diagnostics[-1]['sampling'] == 'xy'
    assert not result.diagnostics[-1]['dense_fallback']


def test_xy_features_keep_reversals_and_projected_normal_changes():
    assert xy_feature_targets([plane(0),plane(1),plane(0)]) == [0,1,2]
    wall = Plane((0,0,0),(1,0,0),(0,0,-1))
    reverse = Plane((0,0,0),(-1,0,0),(0,0,-1))
    assert xy_feature_targets([wall,reverse,wall]) == [0,1,2]
    assert xy_feature_targets([wall]*5) == [0,4]
    with pytest.raises(ValueError):
        xy_feature_targets([wall],xy_tolerance=0)


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


def test_impossible_keyframe_stops_without_dense_fallback():
    calls = []
    targets = [plane(i*.01) for i in range(5)]
    def ik(t,b):
        calls.append((t.origin[0],b.origin[0]))
        return [[0]]
    class Collision:
        last_failure = 'self collision: arm / chassis'
        def valid(self,q,b):
            return False
    result = plan_mobile_sparse(targets, [[t] for t in targets], ik_solver=ik,
                                collision=Collision().valid)
    assert len(calls) == 1
    assert not result.base_planes
    assert result.candidate_counts == [0,None,None,None,None]
    assert not result.diagnostics[-1]['dense_fallback']
    assert result.target_diagnostics[1] == {'checked':False}
    assert result.target_diagnostics[0]['raw_ik'] == 1
    assert result.target_diagnostics[0]['rejection_reasons'] == {'self collision: arm / chassis':1}


def test_early_failure_maps_original_indices_and_does_not_generate_remaining_regions():
    targets = [plane(0),plane(.1),plane(.2),plane(.2,.1),plane(.2,.2)]
    generated = []
    def layers(i,t):
        generated.append(i)
        return [plane(i)]
    result = plan_mobile_sparse(targets,layers,ik_solver=lambda t,b:[[0]] if t.origin[1] == 0 else [],
                                max_gap=2)
    assert result.candidate_counts == [1,None,1,None,0]
    assert generated == [0,2,4]
    assert result.diagnostics[-1]['reason'] == 'keyframe_has_no_feasible_candidate'


def test_skipped_target_failure_still_uses_dense_fallback_and_cached_candidates():
    targets = [plane(i*.01) for i in range(5)]
    calls = []
    def ik(t,b):
        key = (tuple(t.origin),tuple(b.origin))
        calls.append(key)
        return [[0]]
    result = plan_mobile_sparse(targets,[[plane(i*.01),plane(i*.01,.1)] for i in range(5)],
        ik_solver=ik,collision=lambda q,b: not (b.origin[0] == .02 and b.origin[1] == 0))
    assert len(result.base_planes) == 5
    assert result.diagnostics[-1]['dense_fallback']
    assert len(calls) == len(set(calls))


def test_mobile_region_uses_calibrated_arm_origin_and_all_original_targets():
    from motion_toolbox.mobile_planning import plan_mobile_robot_path
    from motion_toolbox.stationary_region import StationaryRegion
    class Solver:
        arm_in_base = Plane((.275,.2,1.03),(-1,0,0),(0,-1,0))
        def __call__(self,t,b):
            return [[0]]
    solver = Solver()
    targets = [Plane((i*.01,0,.1),(1,0,0),(0,0,-1)) for i in range(11)]
    result = plan_mobile_robot_path(targets, [], dict(placement_region=True,sparse=True),
                                    ik_solver=solver)
    assert len(result['base_planes']) == len(result['configurations']) == 11
    for t,b in zip(targets,result['base_planes']):
        m = StationaryRegion([t],solver.arm_in_base,projected=True).metrics(b)
        assert m['geometry_valid']
        assert m['max_projected_distance'] <= 1.75+1e-9
        assert m['arm_origin'][1] < 0


def test_sparse_interpolation_cannot_bypass_placement_rules():
    targets = [plane(i*.01) for i in range(3)]
    layers = [[plane(0,-.1)], [plane(.01,.1)], [plane(.02,-.1)]]
    result = plan_mobile_sparse(targets,layers,ik_solver=lambda t,b:[[0]],
        base_valid=lambda t,b: b.origin[1] > 0 if t.origin[0] == .01 else b.origin[1] < 0)
    assert result.diagnostics[-1]['dense_fallback']
    assert result.base_planes[1].origin[1] > 0

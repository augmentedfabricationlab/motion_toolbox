import numpy as np
import pytest
from motion_toolbox.base_planning import find_stationary_base
from motion_toolbox.geometry import Plane
from motion_toolbox.stationary_region import StationaryRegion


def problem(n=6):
    targets = [Plane((i*.01, 0, 1), (1, 0, 0), (0, 0, -1)) for i in range(n)]
    base = Plane((0, -1, 0), (1, 0, 0), (0, 1, 0))
    region = StationaryRegion(targets, Plane.world_xy(), projected=True)
    return targets, base, region


def plan(layers, fast=True, collision=None, **kwargs):
    targets, base, region = problem(len(layers))
    return find_stationary_base(targets, [base], objective='heuristic',
        placement_region=region, fast_validation=fast, collision=collision,
        ik_solver=lambda t, b: layers[int(round(t.origin[0]*100))], **kwargs)


@pytest.mark.parametrize('seed', range(12))
@pytest.mark.parametrize('start', [None, [0., 0.]])
def test_fast_matches_exhaustive_cost_path_and_ties(seed, start):
    rng = np.random.RandomState(seed)
    layers = [rng.uniform(-1, 1, (12, 2)).tolist() for _ in range(6)]
    collision = lambda q, b: q[0] > -.3
    eager = plan(layers, False, collision, current_pose=start, count_paths=False)
    fast = plan(layers, True, collision, current_pose=start, count_paths=False)
    assert fast.configurations == eager.configurations
    assert fast.cost == pytest.approx(eager.cost)
    assert bool(fast.base_planes) == bool(eager.base_planes)
    assert fast.counts_complete is False and fast.candidate_counts == []
    assert fast.ik_option_count is None
    assert eager.counts_complete is True


@pytest.mark.parametrize('build_path', [False, True])
def test_clear_path_skips_alternatives_and_does_not_repeat_checks(build_path):
    calls = []
    layers = [[[float(j), float(i)] for j in range(10)] for i in range(6)]
    result = plan(layers, collision=lambda q, b: calls.append(tuple(q)) or True,
                  build_path=build_path)
    assert len(calls) == len(set(calls)) == 6
    assert result.path_search_count == int(build_path)


@pytest.mark.parametrize('fast', [True, False])
def test_count_paths_forces_exact_counts(fast):
    result = plan([[[0.], [1.]]] * 4, fast, count_paths=True)
    assert result.counts_complete
    assert result.candidate_counts == [2]*4
    assert result.ik_option_count == result.path_count == 16


@pytest.mark.parametrize('collision', [None, lambda q, b: q[0] != 1.])
def test_disconnection_keeps_selected_base(collision):
    layers = [[[0.], [1.]], [[8.], [9.]]]
    eager, fast = [plan(layers, mode, collision, count_paths=False) for mode in (False, True)]
    assert fast.base_planes and eager.base_planes
    assert fast.configurations == eager.configurations == []
    assert fast.selected_target_planes == eager.selected_target_planes == []
    assert fast.diagnostics[-1]['reason'] == eager.diagnostics[-1]['reason'] == 'joint_step_disconnected'


@pytest.mark.parametrize('layers,collision,ranges,reason', [
    ([[[0.]], []], None, None, 'no_ik'),
    ([[[0.]], [[3.]]], None, [[-1, 1]], 'joint_limits'),
    ([[[0.]], [[1.], [2.]]], lambda q, b: q[0] == 0, None, 'collision'),
])
def test_failure_classification_and_details(layers, collision, ranges, reason):
    eager = plan(layers, False, collision, joint_ranges=ranges)
    fast = plan(layers, True, collision, joint_ranges=ranges)
    assert not fast.base_planes
    assert fast.diagnostics[-1]['reason'] == eager.diagnostics[-1]['reason'] == reason
    a, b = [p.diagnostics[-1]['failed_target_details'] for p in (fast, eager)]
    for key in ('target_index', 'raw_ik', 'within_joint_limits', 'collision_free', 'rejection_reasons'):
        assert a[key] == b[key]


@pytest.mark.parametrize('fail', [False, True])
def test_stationary_restores_cpu_policy(monkeypatch, fail):
    from motion_toolbox import execution
    from test_execution import Controller
    controller = Controller()
    monkeypatch.setattr(execution, '_controller', lambda: controller)
    if fail:
        with pytest.raises(ValueError):
            plan([[[float('nan')]]])
    else:
        plan([[[0.]]])
    assert controller.states == [(1, 3, 2), (1, 2, 2)]


@pytest.mark.parametrize('fast', [True, False])
def test_selected_tcp_tracks_rotation_after_collision_pruning(fast):
    targets, base, region = problem(3)
    def ik(target, base):
        angle = np.arctan2(-target.xaxis[2], target.xaxis[0]) % (2*np.pi)
        return [[float(round(angle/(np.pi/2)))]]
    result = find_stationary_base(targets, [base], current_pose=[2.],
        objective='heuristic', placement_region=region, fast_validation=fast,
        ik_solver=ik, rotation_mode='n_steps', rotation_steps=4,
        collision=lambda q,b: q[0] in (1., 3.), count_paths=False)
    assert result.configurations == [[1.]]*3
    assert result.selected_tcp_rotations == pytest.approx([np.pi/2]*3)
    for target, selected in zip(targets, result.selected_target_planes):
        np.testing.assert_allclose(selected.matrix, target.rotated_z(np.pi/2).matrix)

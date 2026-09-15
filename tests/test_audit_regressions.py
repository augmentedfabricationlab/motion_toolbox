import math
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.planning import calculate_partial_trajectory
from motion_toolbox.rolling import RollingPlanner
from motion_toolbox.utilities.static_planning import run_job, run_all
from motion_toolbox.base_planning import find_stationary_base


def test_target_diagnostics_distinguish_ik_limits_and_collisions(monkeypatch):
    monkeypatch.setenv('TOOLBOX_RECORDING', '0')
    targets = [Plane((i, 0, 0), (1, 0, 0), (0, 1, 0)) for i in range(3)]
    def solver(target, base):
        return [[], [[2.0]], [[0.5]]][int(target.origin[0])]
    class Checker:
        last_failure = None
        def is_valid(self, q, base):
            self.last_failure = 'environment collision: wrist / collision_meshes[0]'
            return False
    result = calculate_partial_trajectory(None, targets, ik_solver=solver,
        joint_ranges=[[0, 1]], collision=Checker().is_valid)
    assert result['unreachable_points'] == [0, 1, 2]
    first, second, third = result['target_diagnostics']
    assert first['raw_ik'] == 0
    assert second['raw_ik'] == 1 and second['within_joint_limits'] == 0
    assert third['within_joint_limits'] == 1 and third['collision_free'] == 0
    assert third['collision_checks'] == 1
    assert third['rejection_reasons'] == {'environment collision: wrist / collision_meshes[0]': 1}
    assert result['timings']['graph_seconds'] == 0
    assert result['timings']['candidate_seconds'] >= result['timings']['collision_seconds']


def test_precomputed_job_applies_limits_and_rejects_unused_options():
    job = dict(current_pose=[0], solutions=[[[.1], [.2]]], planning={'joint_ranges': [[.15, .3]]})
    assert run_job(job)['configurations'] == [[.2]]
    job['planning']['rotaton_steps'] = 3
    with pytest.raises(ValueError, match='Unknown planning options'):
        run_job(job)


@pytest.mark.parametrize('limits', [[[float('nan'), 1]], [[2, 1]], [[0, 1, 2]]])
def test_ranges_validated_even_without_ik(limits):
    with pytest.raises(ValueError):
        calculate_partial_trajectory([0], [Plane.world_xy()], ik_solver=lambda t, b: [], joint_ranges=limits)


def test_explicit_start_base_used_in_first_edge():
    seen = []
    base = Plane((1, 0, 0), (1, 0, 0), (0, 1, 0))
    result = calculate_partial_trajectory([0], [Plane.world_xy()], base_planes=[base],
        start_base=Plane.world_xy(), ik_solver=lambda t,b: [[.1]],
        transition_check=lambda q0,b0,q1,b1: seen.append((b0.origin[0], b1.origin[0])) or True)
    assert result['configurations']
    assert seen == [(0, 1)]


def test_rolling_uses_previous_predicted_base_and_skips_full_buffer():
    bases = [Plane((i, 0, 0), (1, 0, 0), (0, 1, 0)) for i in range(4)]
    calls, edges = [], []
    def ik(t,b):
        calls.append(1)
        return [[.1]]
    planner = RollingPlanner([Plane.world_xy()]*4, lookahead=2, buffer_size=1, ik_solver=ik,
                             transition_check=lambda q0,b0,q1,b1: edges.append((b0.origin[0],b1.origin[0])) or True)
    planner.update(-1, [0], bases)
    total = len(calls)
    planner.update(-1, [0], bases)
    assert len(calls) == total
    edges.clear()
    planner.update(0, [.1], bases)
    assert edges[0] == (0, 1)
    with pytest.raises(ValueError):
        planner.update(0, [.1], bases[:2])


def test_stationary_search_honors_custom_edge_filter():
    r = find_stationary_base([Plane.world_xy()], [Plane.world_xy()], [0],
                              ik_solver=lambda t,b: [[.1]], edge_valid=lambda *a: False)
    assert not r.configurations

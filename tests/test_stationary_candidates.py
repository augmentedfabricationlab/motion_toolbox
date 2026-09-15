import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.base_planning import stationary_base_candidates, stationary_base_guesses, find_stationary_base, plan_mobile_base


def test_generated_grid_covers_bounds_and_can_feed_both_searches():
    targets = [Plane.world_xy(), Plane((1,0,0),(1,0,0),(0,1,0))]
    bases = stationary_base_candidates(targets, margin=0, spacing=.5, yaw_steps=1)
    assert np.allclose([b.origin[0] for b in bases], [0,.5,1])
    def ik(t,b):
        return [[float(t.origin[0]-b.origin[0])]]
    fixed = find_stationary_base(targets, bases, [0], ik_solver=ik)
    mobile = plan_mobile_base(targets, [bases]*len(targets), ik_solver=ik, max_base_step=1)
    assert len(fixed.base_planes) == 1
    assert len(mobile.base_planes) == len(targets)


@pytest.mark.parametrize('kwargs', [{'spacing':0}, {'margin':-1}, {'yaw_steps':1.5}])
def test_invalid_search_domain_rejected(kwargs):
    with pytest.raises(ValueError):
        stationary_base_candidates([Plane.world_xy()], **kwargs)


def test_unseeded_stationary_search_checks_only_printing_transitions():
    edges = []
    targets = [Plane.world_xy(), Plane((1,0,0),(1,0,0),(0,1,0))]
    result = find_stationary_base(targets, [Plane.world_xy()],
        ik_solver=lambda t,b: [[10+float(t.origin[0])]],
        transition_check=lambda q0,b0,q1,b1: edges.append((q0,q1)) or True)
    assert result.configurations == [[10], [11]]
    assert result.cost == pytest.approx(1)
    assert edges == [([10], [11])]


def test_max_paths_prefers_alternatives_over_shorter_travel():
    targets = [Plane.world_xy(), Plane((1,0,0),(1,0,0),(0,1,0))]
    bases = [Plane.world_xy(), Plane((2,0,0),(1,0,0),(0,1,0))]
    def ik(t, b):
        return [[0]] if b.origin[0] == 0 else [[float(t.origin[0])], [float(t.origin[0])+0.2]]
    shortest = find_stationary_base(targets, bases, ik_solver=ik)
    robust = find_stationary_base(targets, bases, ik_solver=ik, objective='max_paths')
    assert shortest.base_plane.origin[0] == 0
    assert robust.base_plane.origin[0] == 2
    assert robust.path_count == 4
    assert robust.cost > shortest.cost


def test_path_counts_include_only_connected_collision_free_sequences():
    targets = [Plane.world_xy()] * 3
    result = find_stationary_base(targets, [Plane.world_xy()], ik_solver=lambda t,b: [[0], [1], [2]],
        collision=lambda q,b: q != [2],
        transition_check=lambda q0,b0,q1,b1: q0 == q1, objective='max_paths')
    assert result.path_count == 2  # 000 and 111, not all eight combinations
    assert result.candidate_counts == [2, 2, 2]
    seeded = find_stationary_base(targets, [Plane.world_xy()], [0], ik_solver=lambda t,b: [[0], [1]],
        transition_check=lambda q0,b0,q1,b1: q0 == q1, objective='max_paths')
    assert seeded.path_count == 1


def test_rotation_expands_sampled_paths_and_deduplicates_solutions():
    def ik(t,b):
        return [[round(float(t.xaxis[0]), 6)]]
    fixed = find_stationary_base([Plane.world_xy()], [Plane.world_xy()], ik_solver=ik, objective='max_paths')
    rotated = find_stationary_base([Plane.world_xy()], [Plane.world_xy()], ik_solver=ik,
        rotation_mode='n_steps', rotation_steps=4, objective='max_paths')
    assert fixed.path_count == 1
    assert rotated.path_count == 3  # +1, 0, -1; the two zero solutions coincide


@pytest.mark.parametrize('ik,collision,transition,reason', [
    (lambda t,b: [], None, None, 'no_ik'),
    (lambda t,b: [[0]], lambda q,b: False, None, 'joint_limits_or_collision'),
    (lambda t,b: [[0]], None, lambda *args: False, 'blocked_transitions'),
])
def test_stationary_failure_diagnostics(ik, collision, transition, reason):
    result = find_stationary_base([Plane.world_xy()]*2, [Plane.world_xy()], ik_solver=ik,
        collision=collision, transition_check=transition, objective='max_paths')
    assert result.configurations == []
    assert result.path_count == 0
    assert result.diagnostics[0]['reason'] == reason


def test_straight_wall_guesses_both_sides_and_accounts_for_mounting():
    targets = [Plane((x, 0, 1), (1,0,0), (0,0,1)) for x in (-.5, 0, .5)]
    mount = Plane((.275, 0, 1), (1,0,0), (0,1,0))
    guesses = stationary_base_guesses(targets, arm_in_base=mount, distance=.8, yaw_steps=4)
    arm_origins = [(base.matrix @ mount.matrix)[:3, 3] for base in guesses]
    assert any(np.allclose(p, (0, .8, 1)) for p in arm_origins)
    assert any(np.allclose(p, (0, -.8, 1)) for p in arm_origins)
    assert all(base.origin[2] == 0 for base in guesses)
    assert all(np.all(np.abs(base.origin[:2]) <= (2, 1.5)) for base in guesses)


def test_arc_guess_includes_equidistant_center_and_is_translation_invariant():
    def make_targets(shift):
        return [Plane(np.array((np.cos(a), np.sin(a), 1))+shift, (1,0,0), (0,1,0))
                for a in np.linspace(-1, 1, 11)]
    guesses = stationary_base_guesses(make_targets(np.zeros(3)), yaw_steps=1)
    shift = np.array((1000, -3000, 0))
    translated = stationary_base_guesses(make_targets(shift), yaw_steps=1)
    assert any(np.allclose(base.origin, (0,0,0), atol=1e-8) for base in guesses)
    # Near-zero facing vectors can select different headings; compare origins.
    for base in guesses:
        assert any(np.allclose(base.origin, moved.origin-shift, atol=1e-7) for moved in translated)


def test_guesses_can_recover_a_solution_between_grid_points_without_losing_grid():
    targets = [Plane((.25, 0, 1), (1,0,0), (0,1,0))]
    grid = stationary_base_candidates(targets, margin=1, spacing=2, yaw_steps=1)
    guesses = stationary_base_guesses(targets, yaw_steps=1)
    combined = stationary_base_candidates(targets, margin=1, spacing=2, yaw_steps=1,
                                          initial_guesses=guesses+grid[:1])
    def ik(t, b):
        return [[0]] if np.linalg.norm(t.origin[:2]-b.origin[:2]) < .01 else []
    assert not find_stationary_base(targets, grid, ik_solver=ik).configurations
    assert find_stationary_base(targets, combined, ik_solver=ik).configurations
    keys = [tuple(np.round(b.matrix.ravel(), 9)) for b in combined]
    assert len(keys) == len(set(keys))
    assert all(tuple(np.round(b.matrix.ravel(), 9)) in keys for b in grid)


@pytest.mark.parametrize('targets,kwargs', [([], {}), ([Plane.world_xy()], {'distance':0}),
    ([Plane.world_xy()], {'yaw_steps':0}), ([Plane.world_xy()], {'margin':-1})])
def test_invalid_guess_inputs(targets, kwargs):
    with pytest.raises(ValueError):
        stationary_base_guesses(targets, **kwargs)


def test_configuration_ranking_builds_only_one_path_and_reuses_winner_ik(monkeypatch):
    import motion_toolbox.base_planning as planning
    targets = [Plane.world_xy()] * 3
    bases = [Plane((x,0,0),(1,0,0),(0,1,0)) for x in (0,1,2)]
    calls, searches = [], []
    original = planning.shortest_path
    def shortest(layers, **kwargs):
        searches.append(layers)
        assert kwargs.get('edge_valid') is None
        return original(layers, **kwargs)
    def ik(t,b):
        calls.append(b.origin[0])
        return [[0],[1]] if b.origin[0] == 1 else [[0]]
    monkeypatch.setattr(planning, 'shortest_path', shortest)
    result = planning.find_stationary_base(targets, bases, ik_solver=ik, objective='ik_options')
    assert result.base_plane.origin[0] == 1
    assert result.ik_option_count == 8
    assert result.path_search_count == len(searches) == 1
    assert len(calls) == 9  # No repeated IK for the selected base.
    assert sum(d['path_checked'] for d in result.diagnostics) == 1


def test_configuration_ranking_stops_at_first_unreachable_target_and_can_skip_path(monkeypatch):
    import motion_toolbox.base_planning as planning
    calls = []
    def ik(t,b):
        calls.append(b.origin[0])
        return [] if b.origin[0] == 0 else [[0]]
    def forbidden(*args, **kwargs):
        raise AssertionError('Path search must not run')
    monkeypatch.setattr(planning, 'shortest_path', forbidden)
    result = planning.find_stationary_base([Plane.world_xy()]*3,
        [Plane.world_xy(), Plane((1,0,0),(1,0,0),(0,1,0))], ik_solver=ik,
        objective='ik_options', build_path=False)
    assert calls == [0,1,1,1]
    assert result.base_plane.origin[0] == 1
    assert result.path_search_count == 0
    assert result.diagnostics[0]['targets_checked'] == 1


def test_winner_retained_if_its_single_path_search_is_disconnected():
    targets = [Plane.world_xy(), Plane((1,0,0),(1,0,0),(0,1,0))]
    result = find_stationary_base(targets, [Plane.world_xy()],
        ik_solver=lambda t,b: [[float(t.origin[0])*10]], objective='ik_options')
    assert result.base_planes
    assert result.configurations == []
    assert result.path_search_count == 1
    assert result.diagnostics[0]['reason'] == 'joint_step_disconnected'


def test_failure_details_distinguish_limits_from_collisions_without_rechecking():
    class Checker:
        calls = 0
        last_failure = None
        def check(self, q, base):
            self.calls += 1
            self.last_failure = 'environment collision: wrist / collision_meshes[0]'
            return False
    checker = Checker()
    limited = find_stationary_base([Plane.world_xy()], [Plane.world_xy()],
        ik_solver=lambda t,b: [[2]], joint_ranges=[[-1,1]], collision=checker.check, objective='ik_options')
    detail = limited.diagnostics[0]['failed_target_details']
    assert limited.diagnostics[0]['reason'] == 'joint_limits'
    assert detail['raw_ik'] == 1 and detail['within_joint_limits'] == 0
    assert checker.calls == 0
    blocked = find_stationary_base([Plane.world_xy()], [Plane.world_xy()],
        ik_solver=lambda t,b: [[0],[.5]], joint_ranges=[[-1,1]], collision=checker.check, objective='ik_options')
    detail = blocked.diagnostics[0]['failed_target_details']
    assert blocked.diagnostics[0]['reason'] == 'collision'
    assert detail['within_joint_limits'] == 2 and detail['collision_free'] == 0
    assert detail['rejection_reasons'] == {'environment collision: wrist / collision_meshes[0]': 2}
    assert checker.calls == 2

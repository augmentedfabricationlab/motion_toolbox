import math
import numpy as np
import pytest

from motion_toolbox.base_planning import find_stationary_base
from motion_toolbox.adaptive_stationary import find_adaptive_stationary_base
from motion_toolbox.kinematics.solver import URKinematics
from motion_toolbox.geometry import Plane
from test_stationary_fast import problem


Q = [0., -1., 1., -.5, 1., 0.]


class BranchSolver(URKinematics):
    def __init__(self, rows):
        super().__init__()
        self.rows = rows

    def __call__(self, target, base):
        return self.rows[int(round(target.origin[0]*100))]


@pytest.fixture(params=['fast', 'eager', 'counted', 'ik_options', 'joint_travel', 'max_paths', 'adaptive'])
def plan(request):
    def run(rows, **options):
        targets, base, region = problem(len(rows))
        solver = options.pop('solver', BranchSolver(rows))
        options = dict(joint_ranges=[[-7, 7]]*6, periodic=[False]*6, **options)
        if request.param == 'adaptive':
            return find_adaptive_stationary_base(targets, ik_solver=solver,
                arm_in_base=Plane.world_xy(), candidate_planes=[base], rotation_steps=1,
                probe_rotations=1, **options)['found']
        mode = request.param
        if mode in ('fast', 'eager', 'counted', 'ik_options'):
            options['count_paths'] = mode == 'counted'
        return find_stationary_base(targets, [base], ik_solver=solver, placement_region=region,
            objective='heuristic' if mode in ('fast', 'eager', 'counted') else mode,
            fast_validation=mode != 'eager', **options)
    return run


@pytest.mark.parametrize('joint,reason', [(2, 'elbow_branch_change'), (4, 'wrist_branch_change')])
def test_small_step_flip_is_rejected(plan, joint, reason):
    changed = Q.copy()
    changed[joint] *= -1
    result = plan([[Q], [changed]])
    assert result.configurations == []
    assert result.configuration_branch_check_applied
    assert result.edge_rejection_reasons[reason] > 0


def test_shoulder_flip_is_rejected(plan):
    before = [0., -math.pi/2-.3, .2, -.1, 1., 0.]
    after = before.copy()
    after[1] += .4
    result = plan([[before], [after]])
    assert not result.configurations
    assert result.edge_rejection_reasons['shoulder_branch_change'] > 0


def test_start_constrains_alternative_path(plan):
    flipped = Q.copy()
    flipped[2] *= -1
    result = plan([[flipped, Q], [flipped, Q]], current_pose=Q)
    np.testing.assert_allclose(result.configurations, [Q, Q])
    assert result.selected_configuration_branch == list(URKinematics().configuration_branch(Q))
    assert not plan([[flipped]], current_pose=Q).configurations


@pytest.mark.parametrize('joint', [2, 4])
def test_ambiguous_nodes_and_start_are_rejected(plan, joint):
    singular = Q.copy()
    singular[joint] = 0.
    flipped = Q.copy()
    flipped[joint] *= -1
    for rows in ([[singular]], [[Q], [singular], [flipped]]):
        result = plan(rows)
        assert not result.configurations
        assert result.edge_rejection_reasons['ambiguous_configuration_branch'] > 0
    result = plan([[Q]], current_pose=singular)
    assert not result.configurations
    assert 'boundary' in result.initial_state_failure


def test_full_turn_crossing_is_rejected(plan):
    changed = Q.copy()
    changed[2] += 2*math.pi-.4
    result = plan([[Q], [changed]], max_joint_step=7.)
    assert not result.configurations
    assert result.edge_rejection_reasons['branch_boundary_crossing'] > 0


def test_collision_pruning_preserves_original_branch_indices(plan):
    alternate, flipped, singular = Q.copy(), Q.copy(), Q.copy()
    alternate[0] = .1
    flipped[2] *= -1
    singular[4] = 0.
    result = plan([[singular, Q, alternate, flipped], [singular, Q, alternate], [flipped, alternate]],
                  collision=lambda q, b: q[0] != 0. or q[2] < 0.)
    np.testing.assert_allclose(result.configurations, [alternate]*3)


def test_lazy_collision_replanning_retains_branch_constraints(plan):
    blocked, alternate, flipped, singular = [Q.copy() for _ in range(4)]
    blocked[5], alternate[5] = .1, .2
    flipped[2] *= -1
    singular[4] = 0.
    result = plan([[singular, flipped, blocked, alternate]]*3, current_pose=Q,
                  collision=lambda q, b: q[5] != .1)
    np.testing.assert_allclose(result.configurations, [alternate]*3)
    assert result.selected_configuration_branch == list(URKinematics().configuration_branch(Q))


def test_custom_solver_without_classifier_retains_previous_behavior(plan):
    flipped = Q.copy()
    flipped[2] *= -1
    rows = [[Q], [flipped]]
    result = plan(rows, solver=lambda t, b: rows[int(round(t.origin[0]*100))])
    np.testing.assert_allclose(result.configurations, [Q, flipped])
    assert not result.configuration_branch_check_applied


def test_reachability_only_does_not_claim_branch_validation(plan):
    result = plan([[Q], [Q]], build_path=False)
    # joint_travel/max_paths always construct a path under their existing API.
    if not result.configurations:
        assert not result.configuration_branch_check_applied


def test_exact_counts_exclude_ambiguous_nodes_and_cross_branch_edges():
    targets, base, region = problem(2)
    singular, flipped = Q.copy(), Q.copy()
    singular[4] = 0.
    flipped[2] *= -1
    result = find_stationary_base(targets, [base], ik_solver=BranchSolver([[singular, Q, flipped]]*2),
        placement_region=region, objective='heuristic', count_paths=True, joint_ranges=[[-3, 3]]*6)
    assert result.path_count == 2


def test_periodic_alias_wraps_but_bounded_turn_is_rejected():
    from motion_toolbox.configuration_branch import shortest_branch_path
    alias = Q.copy()
    alias[2] -= 2*math.pi
    result, _ = shortest_branch_path([[Q], [alias]], solver=URKinematics(), periodic=[True]*6)
    np.testing.assert_allclose(result.configurations, [Q, Q], atol=1e-12)
    result, info = shortest_branch_path([[Q], [alias]], solver=URKinematics(), max_step=7.)
    assert not result.configurations
    assert info['edge_rejection_reasons']['branch_boundary_crossing'] > 0


def test_custom_edge_callback_keeps_original_indices_after_branch_pruning():
    targets, base, _ = problem(2)
    singular, alternate = Q.copy(), Q.copy()
    singular[4] = 0.
    alternate[0] = .1
    seen = []
    def edge(i, a, b):
        seen.append((i, a, b))
        return a == b == 2
    result = find_stationary_base(targets, [base],
        ik_solver=BranchSolver([[singular, Q, alternate]]*2), edge_valid=edge)
    np.testing.assert_allclose(result.configurations, [alternate]*2)
    assert (1, 2, 2) in seen
    assert all(a > 0 and b > 0 for _, a, b in seen)

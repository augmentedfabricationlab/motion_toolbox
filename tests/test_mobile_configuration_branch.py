import math
import numpy as np
import pytest

from motion_toolbox.kinematics.solver import URKinematics
from motion_toolbox.kinematics.ur import forward_kinematics, inverse_kinematics
from motion_toolbox.mobile_base_workflow import validate_base_path
from test_mobile_base_workflow import fixture, World


Q = [0., -1., 1., -.5, 1., 0.]


class BranchSolver(URKinematics):
    def __init__(self, rows):
        super().__init__()
        self.rows = rows

    def __call__(self, target, base):
        return self.rows[int(round(target.origin[0]*10))]


def plan(rows, **options):
    targets, bases = fixture()
    return validate_base_path(targets[:len(rows)], bases[:len(rows)],
        solver=BranchSolver(rows), world=World(), joint_ranges=[[-7,7]]*6,
        periodic=[False]*6, rotation_steps=1, **options)


@pytest.mark.parametrize('joint,reason', [(2,'elbow_branch_change'), (4,'wrist_branch_change')])
def test_small_step_flip_is_rejected(joint, reason):
    changed = Q.copy()
    changed[joint] *= -1
    result = plan([[Q], [changed]])
    assert not result['fabrication_validated']
    assert result['configurations'] == []
    assert result['disconnected_target'] == 1
    assert result['edge_rejection_reasons'][reason] > 0


def test_alternative_branch_preserving_path_and_start_are_used():
    changed = Q.copy()
    changed[2] *= -1
    result = plan([[changed, Q], [changed, Q]], current_pose=Q)
    assert result['fabrication_validated']
    np.testing.assert_allclose(result['configurations'], [Q,Q])
    assert result['configuration_branch_check_applied']


def test_shoulder_flip_is_rejected_with_small_joint_steps():
    before = [0., -math.pi/2-.3, .2, -.1, 1., 0.]
    after = before.copy()
    after[1] += .4
    result = plan([[before], [after]])
    assert not result['fabrication_validated']
    assert result['edge_rejection_reasons']['shoulder_branch_change'] > 0


@pytest.mark.parametrize('joint', [2,4])
def test_singular_node_cannot_bridge_branches(joint):
    singular = Q.copy()
    singular[joint] = 0.
    changed = Q.copy()
    changed[joint] *= -1
    result = plan([[Q], [singular], [changed]])
    assert not result['fabrication_validated']
    assert result['edge_rejection_reasons']['ambiguous_configuration_branch'] > 0
    assert not plan([[singular]])['fabrication_validated']
    assert 'boundary' in plan([[Q]], current_pose=singular)['initial_state_failure']


def test_full_turn_does_not_hide_branch_crossings():
    changed = Q.copy()
    changed[2] += 2*math.pi
    # Wider ranges for this regression (the helper uses +/-7).
    changed[2] -= .4
    result = plan([[Q], [changed]], max_joint_step=7.)
    assert not result['fabrication_validated']
    assert result['edge_rejection_reasons']['branch_boundary_crossing'] > 0


def test_classifier_distinguishes_all_eight_analytic_branches_and_wraps():
    solver = URKinematics()
    solutions = inverse_kinematics(forward_kinematics(Q, solver.parameters), solver.parameters)
    assert len(set(solver.configuration_branch(q) for q in solutions)) == 8
    for q in solutions:
        assert solver.configuration_branch(q) == solver.configuration_branch(np.array(q)+2*math.pi)


def test_collision_replanning_keeps_branch_constraints_with_original_indices():
    alternate = Q.copy()
    alternate[0] = .1
    flipped = Q.copy()
    flipped[2] *= -1

    class Obstructed(World):
        def is_valid(self, q, base, **options):
            self.last_failure = 'tool collision'
            return q[0] != 0. or q[2] < 0

    result = validate_base_path(*fixture(), solver=BranchSolver([[Q,alternate,flipped]]*3),
        world=Obstructed(), joint_ranges=[[-7,7]]*6, periodic=[False]*6,
        rotation_steps=1, _complete_collision_layers=True)
    assert result['fabrication_validated']
    assert result['graph_stats']['graph_solves'] > 1
    np.testing.assert_allclose(result['configurations'], [alternate]*3)


def test_periodic_alias_uses_short_delta_but_bounded_turn_is_rejected():
    alias = Q.copy()
    alias[2] -= 2*math.pi
    targets, bases = fixture()
    result = validate_base_path(targets[:2], bases[:2], solver=BranchSolver([[Q],[alias]]),
        world=World(), joint_ranges=[None]*6, periodic=[True]*6, rotation_steps=1)
    assert result['fabrication_validated']
    np.testing.assert_allclose(result['configurations'], [Q,Q], atol=1e-12)
    assert not plan([[Q],[alias]], max_joint_step=7.)['fabrication_validated']

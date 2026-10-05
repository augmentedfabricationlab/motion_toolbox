import numpy as np
import pytest

from motion_toolbox.geometry import Plane
from motion_toolbox.mobile_base_workflow import generate_base_path, plan_base_path
from test_mobile_base_workflow import Solver, World


class Zero(Solver):
    def __call__(self, target, base):
        return [[0.]*6]


def traversals():
    x = np.r_[np.linspace(0, .8, 25), np.linspace(.8, 0, 25)]
    return [Plane((s, .06*np.sin(5*s), 1), (0, 0, 1), (1, .3*np.cos(5*s), 0))
            for s in x]


@pytest.mark.parametrize('yaw', [0., 37., 170.])
@pytest.mark.parametrize('tangent', [-.3, .3])
def test_fixed_line_heading_reversals_and_unchanged_targets(yaw, tangent):
    targets = traversals()
    result = plan_base_path(targets, solver=Zero(), world=World(),
        joint_ranges=[[-3, 3]]*6, periodic=[False]*6, rotation_steps=1,
        straight_line_motion=True, geometry_mode='legacy', normal_offset=.6, tangent_offset=tangent,
        base_yaw_degrees=yaw, base_yaw_margin_degrees=180.)
    assert result['fabrication_validated'], result['status']
    bases = result['base_planes']
    positions = np.array([b.origin for b in bases])
    direction = np.r_[result['centerline']['direction'], 0.]
    transverse = np.r_[result['centerline']['perpendicular'], 0.]
    np.testing.assert_allclose((positions-positions[0])@transverse, 0., atol=1e-12)
    np.testing.assert_allclose([b.xaxis for b in bases], np.tile(bases[0].xaxis, (len(bases), 1)), atol=1e-12)
    distance = np.diff(positions@direction)
    assert distance.max() > .01 and distance.min() < -.01
    assert result['effective_base_yaw_margin_degrees'] == 0.
    np.testing.assert_array_equal(result['applied_yaw_adjustments_degrees'], 0.)
    nominal = generate_base_path(targets, straight_line_motion=True, geometry_mode='legacy')['base_planes'][0]
    a = np.deg2rad(yaw)
    np.testing.assert_allclose(bases[0].xaxis, np.cos(a)*nominal.xaxis+np.sin(a)*nominal.yaxis)
    for original, selected in zip(targets, result['selected_target_planes']):
        np.testing.assert_allclose(selected.origin, original.origin)
        np.testing.assert_allclose(selected.zaxis, original.zaxis)


@pytest.mark.parametrize('tangent', [-.3, .3])
def test_failed_projection_falls_back_to_adaptive_planner(tangent):
    targets = [Plane((i*.02, 0, 1), (0, 0, 1), (1, 0, 0)) for i in range(33)]
    world = World()
    seen = []
    def base_valid(b, **kwargs):
        seen.append(b)
        # One initial base placement collides. Sliding along the line repairs it.
        return abs(b.origin[0]-(.32-tangent)) > .006
    world.is_base_valid = base_valid
    result = plan_base_path(targets, solver=Zero(), world=world,
        joint_ranges=[[-3, 3]]*6, periodic=[False]*6, rotation_steps=1,
        straight_line_motion=True, normal_offset=.6, tangent_offset=tangent,
        base_yaw_margin_degrees=0.)
    assert result['fabrication_validated'], result['status']
    assert result['straight_line_fallback'] and not result['straight_line_motion']
    assert result['straight_line_attempt']['repair_attempts'] == 0
    assert result['repair_attempts']
    assert np.max(abs(result['applied_offsets'][:, 1]-tangent)) > .006


def test_off_is_identical_and_enabled_does_not_relax_movement_limits():
    targets = traversals()
    kwargs = dict(solver=Zero(), world=World(), joint_ranges=[[-3, 3]]*6,
                  periodic=[False]*6, rotation_steps=1, adapt_offsets=False,
                  normal_offset=.6, tangent_offset=.3, geometry_mode='legacy')
    default = plan_base_path(targets, **kwargs)
    disabled = plan_base_path(targets, straight_line_motion=False, **kwargs)
    np.testing.assert_array_equal([b.matrix for b in default['base_planes']],
                                  [b.matrix for b in disabled['base_planes']])
    assert default['configurations'] == disabled['configurations']
    assert default['path_length'] == disabled['path_length']
    limited = plan_base_path(targets, straight_line_motion=True, max_base_step=.001, **kwargs)
    assert limited['transition_failures']
    assert not limited['fabrication_validated'] and not limited['configurations']
    assert limited['straight_line_fallback']
    assert limited['straight_line_attempt']['transition_failures']


def test_straight_line_overrides_arc_geometry():
    from test_xy_sections import arc, targets
    points, normals = arc(count=33, center=(0, 0))
    result = generate_base_path(targets(points, normals), straight_line_motion=True)
    original = generate_base_path(targets(points, normals))
    bases = result['base_planes']
    assert result['geometry_mode'] == 'straight_line'
    delta = np.array([b.origin-bases[0].origin for b in bases])
    np.testing.assert_allclose(delta@bases[0].xaxis, 0., atol=1e-12)
    np.testing.assert_allclose([b.xaxis for b in bases], np.tile(bases[0].xaxis, (len(bases), 1)))
    direction = np.r_[original['centerline']['direction'], 0.]
    np.testing.assert_allclose(np.array([b.origin for b in bases])@direction,
        np.array([b.origin for b in original['base_planes']])@direction, atol=1e-12)


def test_rejected_straight_candidate_returns_exact_original_validated_path():
    targets = traversals()
    straight = generate_base_path(targets, straight_line_motion=True, normal_offset=.6, tangent_offset=.3,
                                  geometry_mode='legacy')
    base = straight['base_planes'][0]
    world = World()
    world.is_base_valid = lambda b, **kw: abs(np.dot(b.origin-base.origin, base.xaxis)) > 1e-9
    kwargs = dict(solver=Zero(), world=world, joint_ranges=[[-3, 3]]*6,
                  periodic=[False]*6, rotation_steps=1, adapt_offsets=False,
                  normal_offset=.6, tangent_offset=.3, geometry_mode='legacy')
    original = plan_base_path(targets, **kwargs)
    result = plan_base_path(targets, straight_line_motion=True, **kwargs)
    assert original['fabrication_validated'] and result['fabrication_validated']
    assert result['straight_line_fallback'] and result['straight_line_motion_requested']
    assert not result['straight_line_motion']
    assert result['straight_line_attempt']['state_counts'] == {'body_collision': len(targets)}
    np.testing.assert_array_equal([b.matrix for b in result['base_planes']],
                                  [b.matrix for b in original['base_planes']])
    assert result['configurations'] == original['configurations']

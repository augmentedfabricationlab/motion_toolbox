import math
import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.stationary_region import StationaryRegion
from motion_toolbox.base_planning import find_stationary_base


def wall(x=0, y=0, z=1):
    return Plane((x,y,z), (1,0,0), (0,0,-1))  # +Z points along world +Y


def footprint(x=0, y=-1):
    return Plane((x,y,0), (1,0,0), (0,1,0))


def mounting():
    return Plane((0,0,1), (1,0,0), (0,1,0))


def test_wall_region_seeds_near_farthest_possible_point_without_side_toggle():
    region = StationaryRegion([wall(-.5), wall(.5)], mounting())
    bases, seeds, reason = region.candidates(spacing=.3, yaw_steps=4)
    assert not reason and seeds and bases
    best = max(region.metrics(b)['standoff'] for b in seeds)
    assert best == pytest.approx(math.sqrt(1.75**2-.5**2), abs=.001)
    for base in bases:
        metrics = region.metrics(base)
        assert metrics['geometry_valid']
        assert metrics['standoff'] > 0
        assert metrics['max_target_distance'] <= 1.75+1e-9


def test_distance_is_3d_from_rotated_calibrated_arm_origin():
    mount = Plane((.275,0,1), (1,0,0), (0,1,0))
    region = StationaryRegion([wall(z=2)], mount)
    rotated = Plane((0,-1,0), (0,1,0), (-1,0,0))
    metrics = region.metrics(rotated)
    assert metrics['arm_origin'] == pytest.approx([0,-.725,1])
    assert metrics['max_target_distance'] == pytest.approx(math.hypot(.725,1))
    assert metrics['standoff'] == pytest.approx(.725)


def test_exact_side_and_distance_boundaries():
    region = StationaryRegion([wall()], mounting())
    assert region.metrics(footprint(y=-1.75))['geometry_valid']
    assert region.metrics(footprint(y=-1.75001))['too_far_points'] == [0]
    assert region.metrics(footprint(y=0))['wrong_side_points'] == [0]
    assert region.metrics(footprint(y=1))['wrong_side_points'] == [0]


@pytest.mark.parametrize('targets', [
    [wall(-2), wall(2)],
    [wall(z=3)],
    [wall(), Plane((0,0,1), (1,0,0), (0,0,1))],
])
def test_impossible_regions_explain_failure(targets):
    bases, seeds, reason = StationaryRegion(targets, mounting()).candidates()
    assert bases == seeds == []
    assert reason


def test_all_normals_constrain_a_corner_and_rotation_keeps_region():
    targets = [wall(), Plane((0,0,1), (0,1,0), (0,0,1))]  # +Y and +X
    region = StationaryRegion(targets, mounting())
    bases, _, _ = region.candidates()
    assert bases
    rotated = StationaryRegion([t.rotated_z(1.2) for t in targets], mounting())
    for base in bases:
        assert base.origin[0] < 0 and base.origin[1] < 0
        assert rotated.metrics(base)['geometry_valid']


def test_geometry_rejection_precedes_ik_and_collisions_still_reject_valid_geometry():
    targets = [wall()]
    region = StationaryRegion(targets, mounting())
    calls = []
    def ik(t,b):
        calls.append(float(b.origin[1]))
        return [[0]]
    result = find_stationary_base(targets,
        [footprint(y=1), footprint(y=-2), footprint(y=-1.5), footprint(y=-1)],
        ik_solver=ik, placement_region=region, objective='max_paths',
        collision=lambda q,b: b.origin[1] > -1.4)
    assert calls == [-1.5, -1]
    assert result.base_plane.origin[1] == -1
    assert result.diagnostics[0]['reason'] == 'wrong_side'
    assert result.diagnostics[1]['reason'] == 'beyond_reach_limit'
    assert result.diagnostics[2]['reason'] == 'joint_limits_or_collision'


def test_path_options_then_standoff_then_travel_ranking():
    targets = [wall(), wall(.1)]
    region = StationaryRegion(targets, mounting())
    bases = [footprint(y=-.5), footprint(y=-1.5)]
    result = find_stationary_base(targets, bases, ik_solver=lambda t,b: [[0]],
        objective='max_paths', placement_region=region)
    assert result.standoff == pytest.approx(1.5)
    assert result.base_plane.origin[1] == -1.5
    result = find_stationary_base(targets, bases,
        ik_solver=lambda t,b: [[0],[1]] if b.origin[1] == -.5 else [[0]],
        objective='max_paths', placement_region=region)
    assert result.path_count == 4
    assert result.base_plane.origin[1] == -.5


def test_translated_wall_and_lift_height_use_same_region():
    region = StationaryRegion([wall(-.5), wall(.5)], mounting())
    shift = np.array((1500,-2500,2))
    shifted = [Plane(t.origin+shift, t.xaxis, t.yaxis) for t in region.targets]
    moved = StationaryRegion(shifted, mounting(), base_height=2)
    original, _, _ = region.candidates(yaw_steps=1)
    translated, _, _ = moved.candidates(yaw_steps=1)
    assert len(original) == len(translated)
    for base in original:
        assert any(np.allclose(base.origin, b.origin-shift, atol=1e-7) for b in translated)


def test_projected_guess_ignores_height_but_reports_actual_3d_distance():
    region = StationaryRegion([wall(z=3)], mounting(), projected=True)
    bases, seeds, _ = region.candidates()
    assert bases and seeds
    metrics = region.metrics(seeds[0])
    assert metrics['geometry_valid']
    assert metrics['max_projected_distance'] <= 1.75+1e-9
    assert metrics['max_target_distance'] > 1.75


def test_heuristic_checks_body_before_arm_and_moves_inward_with_bounded_ik():
    region = StationaryRegion([wall(), wall(.1)], mounting(), projected=True)
    events = []
    def base_check(b):
        events.append(('base', b.origin[1]))
        return True
    def ik(t,b):
        events.append(('ik', b.origin[1]))
        return [] if b.origin[1] < -1 else [[0]]
    result = find_stationary_base(region.targets, [footprint(y=y) for y in (-1.7,-1.6,-1.2,-.8,-.5)],
        ik_solver=ik, objective='heuristic', placement_region=region, base_collision=base_check)
    assert result.base_plane.origin[1] == -.8
    assert result.heuristic_plane.origin[1] == -1.7
    assert result.validation_attempts == 3
    assert result.path_search_count == 1
    assert events == [('base',-1.7),('ik',-1.7),('base',-1.2),('ik',-1.2),
                      ('base',-.8),('ik',-.8),('ik',-.8)]


def test_body_blocked_candidates_never_trigger_ik():
    region = StationaryRegion([wall()], mounting(), projected=True)
    def forbidden(*args):
        raise AssertionError('IK should not run for a blocked base body')
    result = find_stationary_base(region.targets, [footprint()], ik_solver=forbidden,
        objective='heuristic', placement_region=region, base_collision=lambda b: False)
    assert not result.base_planes
    assert result.validation_attempts == result.path_search_count == 0
    assert result.base_collision_checks == 1


def test_collision_retry_changes_heading_and_checks_failed_target_first():
    targets = [wall(x) for x in (0, .1, .2)]
    region = StationaryRegion(targets, mounting(), projected=True)
    first = footprint(y=-1.3)
    turned = Plane(first.origin, (0,1,0), (-1,0,0))
    events = []
    def ik(t, b):
        events.append((round(float(t.origin[0]), 2), float(b.xaxis[0])))
        return [[float(t.origin[0])]]
    def collision(q, b):
        return q[0] < .2 or b.xaxis[0] == 0
    result = find_stationary_base(targets, [first, turned, footprint(y=-.9)],
        ik_solver=ik, collision=collision, objective='heuristic', placement_region=region)
    assert result.validation_attempts == 2
    assert result.base_plane.xaxis == pytest.approx(turned.xaxis)
    assert events == [(0,1),(.1,1),(.2,1),(.2,0),(0,0),(.1,0)]
    assert result.configurations == [[0], [.1], [.2]]
    assert result.candidate_counts == [1, 1, 1]
    assert result.path_search_count == 1


def test_retry_failure_keeps_original_target_index():
    targets = [wall(x) for x in (0, .1, .2)]
    region = StationaryRegion(targets, mounting(), projected=True)
    result = find_stationary_base(targets, [footprint(y=-1.3), footprint(y=-.9)],
        ik_solver=lambda t,b: [[float(t.origin[0])]],
        collision=lambda q,b: q[0] < .2, objective='heuristic', placement_region=region)
    assert result.diagnostics[1]['targets_checked'] == 1
    assert result.diagnostics[1]['checked_target_indices'] == [2]
    assert result.diagnostics[1]['failed_target_details']['target_index'] == 2

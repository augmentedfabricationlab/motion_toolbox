import itertools
import math
import numpy as np
import pytest
from motion_toolbox.geometry import Plane, as_plane
from motion_toolbox.graph import shortest_path


@pytest.mark.parametrize('count_paths', [True, False])
def test_indexed_path_search_matches_dense_search(count_paths):
    rng = np.random.default_rng(71)
    layers = [rng.uniform(-4,4,(70,3)).tolist() for _ in range(3)]
    mask = np.array([False, True, False])
    limits = np.array([2.5, 2.5, 2.5])
    def allowed(i,a,b):
        return (i+a+b) % 7 != 0
    def dense_edge(i,a,b):
        delta = np.array(layers[i][b])-layers[i-1][a]
        delta[mask] = (delta[mask]+np.pi) % (2*np.pi)-np.pi
        return bool(np.all(np.abs(delta) <= limits)) and allowed(i,a,b)
    indexed = shortest_path(layers, max_step=limits, periodic=mask,
                            edge_valid=allowed, count_paths=count_paths)
    dense = shortest_path(layers, max_step=None, periodic=mask,
                          edge_valid=dense_edge, count_paths=count_paths)
    assert indexed.indices == dense.indices
    assert indexed.cost == pytest.approx(dense.cost)
    assert indexed.path_count == dense.path_count
from motion_toolbox.kinematics.ur import inverse_kinematics, forward_kinematics
from motion_toolbox.kinematics.solver import URKinematics
from motion_toolbox.planning import calculate_partial_trajectory, rotation_offsets
from motion_toolbox.base_planning import find_stationary_base, plan_mobile_base, grid_bases
from motion_toolbox.rolling import RollingPlanner


def test_geometry_does_not_mutate_input():
    x, y = np.array([2., 0, 0]), np.array([1., 1., 0])
    p = Plane([0, 0, 0], x, y)
    np.testing.assert_equal(x, [2, 0, 0])
    np.testing.assert_equal(y, [1, 1, 0])
    np.testing.assert_allclose(p.matrix, np.eye(4))
    np.testing.assert_allclose(as_plane(p.to_dict()).matrix, p.matrix)
    with pytest.raises(ValueError):
        Plane([0, 0, 0], [1, 0, 0], [2, 0, 0])


def test_exact_graph_against_exhaustive():
    rng = np.random.default_rng(14)
    for _ in range(15):
        layers = [rng.normal(size=(4, 3)).tolist() for _ in range(5)]
        start = [0, 0, 0]
        result = shortest_path(layers, start=start, max_step=None, chunk_size=2)
        def cost(path):
            points = [start]+[layer[i] for layer, i in zip(layers, path)]
            return sum(np.linalg.norm(np.array(b)-a) for a, b in zip(points, points[1:]))
        expected = min(cost(path) for path in itertools.product(range(4), repeat=5))
        assert result.cost == pytest.approx(expected)


def test_graph_empty_disconnected_periodic_and_first_edge():
    assert shortest_path([]).cost == 0
    assert not shortest_path([[[0]], []]).configurations
    assert not shortest_path([[[3]]], start=[0], max_step=2).configurations
    assert not shortest_path([[[-3.1]]], start=[3.1], max_step=.2).configurations
    result = shortest_path([[[-3.1]]], start=[3.1], max_step=.2, periodic=[True])
    assert result.cost < .1
    assert result.configurations[0][0] > math.pi
    assert not shortest_path([[[0]], [[.1]]], edge_valid=lambda *x: False).configurations


@pytest.mark.parametrize('count_paths', [False, True])
@pytest.mark.parametrize('seeded', [False, True])
def test_full_turn_index_preserves_bounded_paths(count_paths, seeded):
    rng = np.random.default_rng(42)
    physical = rng.uniform(-3.2, 3.2, (12, 3))
    turns = np.array(list(itertools.product([-2*np.pi, 0, 2*np.pi], repeat=3)))
    layers = []
    for i in range(4):
        layer = (physical[:, None, :] + turns + i*.025).reshape(-1, 3)
        # Unequal bounds and missing windings must stay respected.
        layer = layer[np.all((layer >= [-7, -5, -6]) & (layer <= [5, 7, 6]), axis=1)]
        layers.append(layer[rng.permutation(len(layer))].tolist())
    options = dict(max_step=[2.5, 1.8, 2.8], weights=[.5, 1, 2],
                   count_paths=count_paths, start=layers[0][0] if seeded else None)
    expected = shortest_path(layers, **options)
    actual = shortest_path(layers, revolute_joints=range(3), **options)
    assert actual.cost == pytest.approx(expected.cost)
    assert actual.indices == expected.indices
    assert actual.path_count == expected.path_count


def test_full_turn_index_boundary_and_missing_winding():
    from motion_toolbox.graph import _winding_layer
    previous = np.array([[3.14], [3.14-2*np.pi]])
    current = np.array([[-3.13], [3.15], [9.43]])
    costs, parents, counts = _winding_layer(previous, current, np.zeros(2), [1, 1],
                                          np.ones(1), np.array([.2]), True)
    assert parents[:2].tolist() == [1, 0]
    assert counts == [1, 1, 0]
    assert np.isinf(costs[-1])


def test_ik_fk_roundtrip_and_nonmutating():
    q = [.4, -1.3, 1.1, -.6, .9, .7]
    original = list(q)
    frame = forward_kinematics(q)
    assert q == original
    solutions = inverse_kinematics(frame)
    assert solutions
    assert min(np.linalg.norm((np.array(s)-q+np.pi) % (2*np.pi)-np.pi) for s in solutions) < 1e-6
    for s in solutions:
        np.testing.assert_allclose(forward_kinematics(s).matrix, frame.matrix, atol=1e-7)


def test_tcp_and_base_transforms():
    q = [.2, -1.2, 1.1, -.5, 1.3, .4]
    flange = forward_kinematics(q)
    tool = Plane((.1, .2, .3), (0, 1, 0), (0, 0, 1))
    arm = Plane((.3, .1, 1), (1, 0, 0), (0, 1, 0))
    base = Plane((2, -1, 0), (0, 1, 0), (-1, 0, 0))
    tcp = Plane.from_matrix(base.matrix @ arm.matrix @ flange.matrix @ tool.matrix)
    solutions = URKinematics(tool=tool, arm_in_base=arm)(tcp, base)
    assert min(np.linalg.norm((np.array(s)-q+np.pi) % (2*np.pi)-np.pi) for s in solutions) < 1e-6


def test_partial_rotation_filter_and_collision_base_alignment():
    targets = [Plane.world_xy()]*3
    bases = grid_bases([0, 1, 2], [0])
    seen = []
    def ik(t, b):
        return [[math.atan2(t.xaxis[1], t.xaxis[0])]]
    def valid(q, b):
        seen.append(float(b.origin[0]))
        return q[0] > .1
    result = calculate_partial_trajectory([0], targets, 2, bases, 'n_steps', rotation_steps=4,
        ik_solver=ik, collision=valid, max_joint_step=None)
    assert result['num_nodes_computed'] == 2
    assert len(result['configurations']) == 2
    assert set(seen) == {0, 1}
    assert calculate_partial_trajectory([0], [], ik_solver=ik)['configurations'] == []
    with pytest.raises(ValueError):
        calculate_partial_trajectory([0], targets, enable_collision_check=True)


def test_stationary_requires_every_target_and_mobile_can_change_base():
    targets = [Plane((x, 0, 1), (1, 0, 0), (0, 1, 0)) for x in (0, 1)]
    bases = grid_bases([0, 1], [0])
    def ik(t, b):
        return [[0]] if abs(t.origin[0]-b.origin[0]) < .1 else []
    stationary = find_stationary_base(targets, bases, [0], ik_solver=ik)
    assert not stationary.base_planes
    mobile = plan_mobile_base(targets, [bases, bases], ik_solver=ik, max_base_step=1.1)
    assert [b.origin[0] for b in mobile.base_planes] == [0, 1]
    assert not plan_mobile_base(targets, [bases, bases], ik_solver=ik, max_base_step=.5).base_planes
    assert not plan_mobile_base(targets, [bases, bases], ik_solver=ik, max_base_step=2,
        transition_check=lambda *args: False).base_planes
    assert not plan_mobile_base(targets, [bases, bases], ik_solver=ik, max_base_step=2,
        time_intervals=[1], max_base_speed=.5).base_planes


def test_rolling_buffer_preserves_committed_and_retries_failure():
    targets = [Plane((i*.1, 0, 0), (1, 0, 0), (0, 1, 0)) for i in range(5)]
    state = {'fail': True}
    def ik(t, b):
        return [] if state['fail'] else [[float(t.origin[0])]]
    planner = RollingPlanner(targets, lookahead=3, buffer_size=2, ik_solver=ik)
    assert planner.update(-1, [0]) == []
    state['fail'] = False
    assert [i for i, q in planner.update(-1, [0])] == [0, 1]
    assert planner.update(-1, [0]) == []
    assert [i for i, q in planner.update(0, [0])] == [2]
    assert [i for i, q in planner.update(1, [.1])] == [3]
    with pytest.raises(ValueError):
        planner.update(0, [0])

import numpy as np
import pytest
from motion_toolbox.graph import shortest_path


@pytest.mark.parametrize('seed', range(6))
def test_lazy_nodes_match_eager_optimum_and_original_indices(seed):
    rng = np.random.RandomState(seed)
    layers = [rng.uniform(-1, 1, (8, 2)).tolist() for _ in range(7)]
    allowed = [rng.rand(8) > .35 for _ in layers]
    maps = [np.flatnonzero(mask).tolist() for mask in allowed]
    eager = shortest_path([[layer[j] for j in ids] for layer, ids in zip(layers, maps)],
                          max_step=1.7, count_paths=False, start=[0., 0.])
    calls = []
    stats = {}
    def check(i, j):
        calls.append((i, j))
        return allowed[i][j]
    lazy = shortest_path(layers, node_valid=check, stats=stats, max_step=1.7,
                         count_paths=False, start=[0., 0.])
    assert lazy.cost == pytest.approx(eager.cost)
    assert lazy.indices == [maps[i][j] for i, j in enumerate(eager.indices)]
    assert len(calls) == len(set(calls))
    assert stats['graph_solves'] >= 1


def test_lazy_reports_collision_empty_layer_and_honors_per_layer_steps():
    layers = [[[0.]], [[.1]], [[.2]]]
    blocked = shortest_path(layers, node_valid=lambda i,j:i != 1, count_paths=False)
    assert not blocked.configurations and blocked.failure_layer == 1
    limited = shortest_path(layers, step_limits=[[1.], [.05], [1.]], count_paths=False)
    assert not limited.configurations and limited.failure_layer == 1


def test_lazy_can_skip_unused_collision_checks_without_changing_cost():
    layers = [[[0.], [1.], [2.]] for _ in range(10)]
    checked = []
    result = shortest_path(layers, node_valid=lambda i,j:checked.append((i,j)) or True,
                           count_paths=False)
    assert result.cost == 0
    assert len(checked) == 10
    assert result.indices == [0]*10


def test_lazy_rejects_path_count_request_instead_of_claiming_unchecked_counts():
    with pytest.raises(ValueError, match='count_paths'):
        shortest_path([[[0.]]], node_valid=lambda i,j:True)


def test_certified_collision_groups_preserve_bounded_states_cost_and_ties():
    layers=[[[.1], [.1+2*np.pi], [1.], [1.+2*np.pi]] for _ in range(6)]
    valid=lambda i,j:j in (2,3)
    eager=shortest_path([[p[2],p[3]] for p in layers],start=[.2],max_step=2.5,count_paths=False)
    calls=[];stats={}
    lazy=shortest_path(layers,start=[.2],max_step=2.5,count_paths=False,
        node_valid=lambda i,j:calls.append((i,j)) or valid(i,j),stats=stats,
        node_rejection_group=lambda i,j:[0,1] if j<2 else [2,3])
    assert lazy.configurations==eager.configurations
    assert lazy.cost==eager.cost
    assert lazy.indices==[j+2 for j in eager.indices]
    assert stats['additional_rejections']==6
    assert (0,1) not in calls
    assert all((i,j) in calls for i,j in enumerate(lazy.indices))

import numpy as np
import pytest
from motion_toolbox.graph import shortest_path


@pytest.mark.parametrize('width', [8,96])
@pytest.mark.parametrize('start', [None,[0.,0.]])
@pytest.mark.parametrize('seed', range(5))
def test_cost_ordered_edges_match_exhaustive_graph(width,start,seed):
    rng = np.random.default_rng(seed)
    layers = [rng.integers(-2,3,(width,2)).astype(float).tolist() for _ in range(4)]
    accepted = rng.random((4,width+1,width)) < .2
    calls = [0]
    def edge(i,a,b):
        calls[0] += 1
        return accepted[i,a+1,b]
    exact = shortest_path(layers,start=start,max_step=1.5,edge_valid=edge,count_paths=True)
    exhaustive_calls = calls[0]
    calls[0] = 0
    fast = shortest_path(layers,start=start,max_step=1.5,edge_valid=edge,count_paths=False)
    assert fast.indices == exact.indices
    assert fast.configurations == exact.configurations
    assert fast.cost == exact.cost
    assert calls[0] <= exhaustive_calls


def test_accepting_edges_checks_one_predecessor_per_node_with_stable_ties():
    calls = []
    result = shortest_path([[[0.]]*100]*3,edge_valid=lambda *edge: calls.append(edge) or True,
                           count_paths=False)
    assert result.indices == [0,0,0]
    assert len(calls) == 200

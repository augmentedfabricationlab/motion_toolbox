import numpy as np
import pytest
from motion_toolbox.graph import shortest_path, lazy_shortest_path


@pytest.mark.parametrize('seed', range(12))
def test_lazy_matches_eager_cost_indices_and_failure(seed):
    rng = np.random.default_rng(seed)
    layers = [rng.normal(size=(6, 2)).tolist() for _ in range(8)]
    allowed = rng.random((8, 6, 6)) > .5
    edge = lambda i,a,b: bool(allowed[i,a,b])
    options = dict(max_step=1.5, count_paths=False)
    eager = shortest_path(layers, edge_valid=edge, **options)
    lazy = lazy_shortest_path(layers, edge_valid=edge, **options)
    assert lazy.indices == eager.indices
    assert lazy.cost == eager.cost
    assert lazy.failure_layer == eager.failure_layer
    assert lazy.reachable_indices == eager.reachable_indices


def test_lazy_checks_every_selected_edge_including_start_and_reuses_checks():
    layers = [[[0.], [1.], [2.]] for _ in range(10)]
    calls = []
    def edge(i,a,b):
        calls.append((i,a,b))
        return True
    result = lazy_shortest_path(layers, start=[0.], edge_valid=edge, count_paths=False)
    assert len(result.configurations) == 10
    assert calls == [(0,-1,0)] + [(i,0,0) for i in range(1,10)]

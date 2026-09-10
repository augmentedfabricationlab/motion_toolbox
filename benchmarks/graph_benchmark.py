"""Compare equal-cost implicit DP and materialized NetworkX DAG shortest paths.

Run: python benchmarks/graph_benchmark.py --output benchmarks/results.json
NetworkX is needed only for this reference benchmark.
"""
import argparse
import json
import platform
import statistics
import time
import tracemalloc
from pathlib import Path
import numpy as np
import networkx as nx
from motion_toolbox.graph import shortest_path


def explicit(layers):
    graph = nx.DiGraph()
    graph.add_node('start')
    graph.add_node('end')
    for j in range(len(layers[0])):
        graph.add_edge('start', (0, j), weight=0.)
    for i in range(1, len(layers)):
        for a, q0 in enumerate(layers[i-1]):
            for b, q1 in enumerate(layers[i]):
                graph.add_edge((i-1, a), (i, b), weight=float(np.linalg.norm(q1-q0)))
    for j in range(len(layers[-1])):
        graph.add_edge((len(layers)-1, j), 'end', weight=0.)
    return nx.dijkstra_path_length(graph, 'start', 'end')


def measure(fn):
    elapsed = []
    for _ in range(3):
        start = time.perf_counter()
        cost = fn()
        elapsed.append(time.perf_counter()-start)
    tracemalloc.start()
    fn()
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return dict(cost=cost, median_seconds=statistics.median(elapsed), peak_python_bytes=peak)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rng = np.random.default_rng(7)
    cases = []
    for n, k in [(100, 8), (100, 32), (200, 48)]:
        layers = [rng.normal(size=(k, 6)) for _ in range(n)]
        implicit = measure(lambda: shortest_path(layers, max_step=None).cost)
        materialized = measure(lambda: explicit(layers))
        assert np.isclose(implicit['cost'], materialized['cost'])
        cases.append(dict(targets=n, candidates=k, implicit=implicit, networkx=materialized,
                          speedup=materialized['median_seconds']/implicit['median_seconds']))
    payload = dict(python=platform.python_version(), platform=platform.platform(), numpy=np.__version__,
                   note='Synthetic graph-only benchmark. No IK, collision or real robot timing. Memory is tracemalloc peak.', cases=cases)
    args.output.write_text(json.dumps(payload, indent=2))
    print(json.dumps(payload, indent=2))


if __name__ == '__main__':
    main()

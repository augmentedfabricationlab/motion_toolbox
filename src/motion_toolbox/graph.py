"""Exact shortest path on an implicit layered directed acyclic graph."""
from dataclasses import dataclass
import numpy as np


@dataclass
class GraphResult:
    configurations: list
    indices: list
    cost: float


def shortest_path(layers, *, start=None, weights=None, periodic=None, max_step=2.5,
                  edge_valid=None, chunk_size=128):
    """Minimize summed weighted joint distances over all adjacent-layer edges.

    No random endpoints or materialized graph. Memory is bounded by a block
    of pair costs and one predecessor per candidate. periodic applies only
    to explicitly continuous joints; bounded joints use actual angle deltas.
    edge_valid(layer_index, previous_index, next_index) can reject transitions;
    previous_index=-1 denotes the initial configuration.
    """
    if chunk_size < 1:
        raise ValueError('chunk_size must be positive')
    if not layers:
        return GraphResult([], [], 0.0)
    if any(len(layer) == 0 for layer in layers):
        return GraphResult([], [], float('inf'))
    arrays = [np.asarray(layer, dtype=float) for layer in layers]
    if arrays[0].ndim != 2 or arrays[0].shape[1] == 0:
        raise ValueError('Each layer must be a nonempty matrix')
    n = arrays[0].shape[1]
    if any(a.ndim != 2 or a.shape[1] != n or not np.isfinite(a).all() for a in arrays):
        raise ValueError('Candidate dimensions must match and values must be finite')
    w = np.ones(n) if weights is None else np.asarray(weights, dtype=float)
    periodic = np.zeros(n, dtype=bool) if periodic is None else np.asarray(periodic, dtype=bool)
    limit = np.broadcast_to(np.asarray(np.inf if max_step is None else max_step, dtype=float), (n,))
    if w.shape != (n,) or periodic.shape != (n,) or not np.isfinite(w).all() or np.any(w < 0) or np.any(np.isnan(limit)) or np.any(limit < 0):
        raise ValueError('Invalid weights, periodic mask or step limits')
    parents = []
    if start is None:
        costs = np.zeros(len(arrays[0]))
        parents.append(np.full(len(costs), -1, dtype=int))
        begin = 1
    else:
        previous = np.asarray(start, dtype=float).reshape(1, -1)
        if previous.shape[1] != n or not np.isfinite(previous).all():
            raise ValueError('Invalid start configuration')
        costs = np.zeros(1)
        begin = 0
    for i in range(begin, len(arrays)):
        if i > 0:
            previous = arrays[i-1]
        current = arrays[i]
        next_costs = np.full(len(current), np.inf)
        pred = np.full(len(current), -1, dtype=int)
        for offset in range(0, len(current), chunk_size):
            block = current[offset:offset+chunk_size]
            delta = block[None, :, :] - previous[:, None, :]
            delta[..., periodic] = (delta[..., periodic]+np.pi) % (2*np.pi)-np.pi
            values = costs[:, None] + np.linalg.norm(delta*w, axis=2)
            values[np.any(np.abs(delta) > limit, axis=2)] = np.inf
            if edge_valid is not None:
                for a, b in np.argwhere(np.isfinite(values)):
                    if not edge_valid(i, int(a) if i > 0 else -1, int(offset+b)):
                        values[a, b] = np.inf
            chosen = values.argmin(axis=0)
            selected = values[chosen, np.arange(len(block))]
            next_costs[offset:offset+len(block)] = selected
            pred[offset:offset+len(block)] = chosen
        if not np.isfinite(next_costs).any():
            return GraphResult([], [], float('inf'))
        costs = next_costs
        parents.append(pred)
    index = int(costs.argmin())
    total = float(costs[index])
    indices = [index]
    for i in range(len(arrays)-1, 0, -1):
        index = int(parents[i][index])
        indices.append(index)
    indices.reverse()
    configs = [a[j].copy() for a, j in zip(arrays, indices)]
    # Return the same continuous-joint representatives used by the edge cost.
    ref = np.asarray(start) if start is not None else configs[0]
    for q in configs:
        q[periodic] = ref[periodic] + (q[periodic]-ref[periodic]+np.pi) % (2*np.pi)-np.pi
        ref = q
    return GraphResult([q.tolist() for q in configs], indices, total)

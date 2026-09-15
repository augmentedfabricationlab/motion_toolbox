"""Exact shortest path on an implicit layered directed acyclic graph."""
from motion_toolbox.recording import recorded, event, metric
from dataclasses import dataclass
import numpy as np


@dataclass
class GraphResult:
    configurations: list
    indices: list
    cost: float
    path_count: int = 0


def _winding_layer(previous, current, costs, counts, weights, limits, count_paths):
    """Exact bounded-angle transitions using one predecessor per physical pose.

    With step limits strictly below pi, at most one full-turn representative of
    a given physical pose can reach a particular next state. Joint limits remain
    enforced by the actual states present in the lookup table.
    """
    period = 2*np.pi
    principal = (previous+np.pi) % period-np.pi
    _, first, groups = np.unique(np.round(principal, 10), axis=0,
                                 return_index=True, return_inverse=True)
    if len(first)*2 > len(previous):
        return None
    representatives = principal[first]
    turns = np.rint((previous-representatives[groups])/period).astype(np.int64)
    lower, upper = turns.min(axis=0), turns.max(axis=0)
    sizes = upper-lower+1
    slots = int(np.prod(sizes.astype(object)))
    if slots*len(first) > 1000000:
        return None
    strides = np.cumprod(np.r_[1, sizes[:-1]])
    codes = (turns-lower) @ strides
    if len(np.unique(groups*slots+codes)) != len(previous):
        return None  # Distinct near-identical poses: retain the general solver.
    table = np.full((len(first), slots), -1, dtype=int)
    table[groups, codes] = np.arange(len(previous))
    required = np.rint((current[:,None,:]-representatives[None,:,:])/period).astype(np.int64)
    valid = np.all((required >= lower) & (required <= upper), axis=2)
    codes = (required-lower) @ strides
    indices = table[np.arange(len(first))[None,:], np.clip(codes, 0, slots-1)]
    valid &= indices >= 0
    safe = np.maximum(indices, 0)
    delta = current[:,None,:]-previous[safe]
    valid &= np.all(np.abs(delta) <= limits, axis=2)
    values = costs[safe]+np.linalg.norm(delta*weights, axis=2)
    values[~valid] = np.inf
    best = values.min(axis=1)
    # Preserve the original first-predecessor tie break across physical groups.
    parents = np.where(values == best[:,None], safe, len(previous)).min(axis=1)
    next_counts = [0]*len(current)
    if count_paths:
        previous_counts = np.asarray(counts, dtype=object)
        next_counts = np.where(np.isfinite(values), previous_counts[safe], 0).sum(axis=1).tolist()
    return best, parents, next_counts


@recorded
def shortest_path(layers, *, start=None, weights=None, periodic=None, max_step=2.5,
                  edge_valid=None, chunk_size=128, count_paths=True, revolute_joints=None):
    """Minimize summed weighted joint distances over all adjacent-layer edges.

    No random endpoints or materialized graph. Memory is bounded by a block
    of pair costs and one predecessor per candidate. periodic applies only
    to explicitly continuous joints; bounded joints use actual angle deltas.
    edge_valid(layer_index, previous_index, next_index) can reject transitions;
    previous_index=-1 denotes the initial configuration.
    count_paths=False skips the potentially huge integer path-count calculation.
    revolute_joints identifies angular axes eligible for full-turn indexing;
    it does not make bounded joints periodic or remove their limits.
    """
    if chunk_size < 1:
        raise ValueError('chunk_size must be positive')
    if not layers:
        return GraphResult([], [], 0.0)
    if any(len(layer) == 0 for layer in layers):
        return GraphResult([], [], float('inf'))
    arrays = [np.asarray(layer, dtype=float) for layer in layers]
    metric('graph.layers', len(arrays))
    metric('graph.nodes', sum(len(a) for a in arrays))
    metric('graph.possible_edges', sum(len(a)*len(b) for a, b in zip(arrays, arrays[1:])))
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
        counts = [1] * len(costs)
        parents.append(np.full(len(costs), -1, dtype=int))
        begin = 1
    else:
        previous = np.asarray(start, dtype=float).reshape(1, -1)
        if previous.shape[1] != n or not np.isfinite(previous).all():
            raise ValueError('Invalid start configuration')
        costs = np.zeros(1)
        counts = [1]
        begin = 0
    for i in range(begin, len(arrays)):
        if i > 0:
            previous = arrays[i-1]
        current = arrays[i]
        next_costs = np.full(len(current), np.inf)
        pred = np.full(len(current), -1, dtype=int)
        next_counts = [0] * len(current)
        if (len(previous) >= 64 and edge_valid is None and not periodic.any()
            and revolute_joints is not None and set(revolute_joints) == set(range(n))
            and np.all(limit < np.pi)):
            indexed = _winding_layer(previous, current, costs, counts, w, limit, count_paths)
            if indexed is not None:
                costs, pred, counts = indexed
                if not np.isfinite(costs).any():
                    return GraphResult([], [], float('inf'))
                parents.append(pred)
                continue
        # Full-turn representatives create large layers, but most cannot be
        # connected under bounded joint steps. Index the most selective joint
        # interval first rather than allocating every possible pair distance.
        axes = np.flatnonzero(~periodic & np.isfinite(limit))
        active = np.flatnonzero(np.isfinite(costs))
        if len(previous) >= 64 and len(axes):
            orders = [active[np.argsort(previous[active, axis], kind='stable')] for axis in axes]
            lower, upper = [], []
            for axis, order in zip(axes, orders):
                sorted_values = previous[order, axis]
                lower.append(np.searchsorted(sorted_values, current[:, axis]-limit[axis]-1e-12))
                upper.append(np.searchsorted(sorted_values, current[:, axis]+limit[axis]+1e-12, side='right'))
            lower, upper = np.array(lower), np.array(upper)
            selective = (upper-lower).argmin(axis=0)
            for b, axis_index in enumerate(selective):
                indices = np.sort(orders[axis_index][lower[axis_index,b]:upper[axis_index,b]])
                if not len(indices):
                    continue
                delta = current[b]-previous[indices]
                delta[:, periodic] = (delta[:, periodic]+np.pi) % (2*np.pi)-np.pi
                keep = np.all(np.abs(delta) <= limit, axis=1)
                indices, delta = indices[keep], delta[keep]
                if edge_valid is not None:
                    keep = np.array([edge_valid(i, int(a) if i > 0 else -1, b) for a in indices], dtype=bool)
                    indices, delta = indices[keep], delta[keep]
                if not len(indices):
                    continue
                values = costs[indices] + np.linalg.norm(delta*w, axis=1)
                chosen = int(values.argmin())
                next_costs[b], pred[b] = values[chosen], indices[chosen]
                if count_paths:
                    next_counts[b] = sum(counts[a] for a in indices)
            if not np.isfinite(next_costs).any():
                event('graph.disconnected', layer=i)
                return GraphResult([], [], float('inf'))
            event('graph.layer', layer=i, reachable_nodes=int(np.isfinite(next_costs).sum()),
                  minimum_cost=float(np.min(next_costs)), algorithm='indexed')
            costs, counts = next_costs, next_counts
            parents.append(pred)
            continue
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
            # Python integers preserve exact counts even for very long paths.
            if count_paths:
                for b in range(len(block)):
                    next_counts[offset+b] = sum(counts[a] for a in np.flatnonzero(np.isfinite(values[:, b])))
            chosen = values.argmin(axis=0)
            selected = values[chosen, np.arange(len(block))]
            next_costs[offset:offset+len(block)] = selected
            pred[offset:offset+len(block)] = chosen
        if not np.isfinite(next_costs).any():
            event('graph.disconnected', layer=i)
            return GraphResult([], [], float('inf'))
        event('graph.layer', layer=i, reachable_nodes=int(np.isfinite(next_costs).sum()),
              minimum_cost=float(np.min(next_costs)), algorithm='dense')
        costs = next_costs
        counts = next_counts
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
    return GraphResult([q.tolist() for q in configs], indices, total, sum(counts) if count_paths else 0)

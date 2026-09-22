"""Exact shortest path on an implicit layered directed acyclic graph."""
from motion_toolbox.recording import recorded, event, metric
from dataclasses import dataclass
from time import perf_counter
import numpy as np


@dataclass
class GraphResult:
    configurations: list
    indices: list
    cost: float
    path_count: int = 0
    failure_layer: int = None
    reachable_indices: tuple = ()




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
    # Keep pair temporaries in cache instead of allocating a full layer x pose x joint tensor.
    best_all = np.empty(len(current))
    parents_all = np.empty(len(current), dtype=int)
    counts_all = []
    for offset in range(0, len(current), 128):
        block = current[offset:offset+128]
        required = np.rint((block[:,None,:]-representatives[None,:,:])/period).astype(np.int64)
        valid = np.all((required >= lower) & (required <= upper), axis=2)
        codes = (required-lower) @ strides
        indices = table[np.arange(len(first))[None,:], np.clip(codes, 0, slots-1)]
        valid &= indices >= 0
        safe = np.maximum(indices, 0)
        delta = block[:,None,:]-previous[safe]
        valid &= np.all(np.abs(delta) <= limits, axis=2)
        values = costs[safe]+np.linalg.norm(delta*weights, axis=2)
        values[~valid] = np.inf
        best = values.min(axis=1)
        # Preserve the original first-predecessor tie break across physical groups.
        parents = np.where(values == best[:,None], safe, len(previous)).min(axis=1)
        next_counts = [0]*len(block)
        if count_paths:
            previous_counts = np.asarray(counts, dtype=object)
            next_counts = np.where(np.isfinite(values), previous_counts[safe], 0).sum(axis=1).tolist()
        best_all[offset:offset+len(block)] = best
        parents_all[offset:offset+len(block)] = parents
        counts_all.extend(next_counts)
    return best_all, parents_all, counts_all


@recorded
def shortest_path(layers, *, start=None, weights=None, periodic=None, max_step=2.5,
                  edge_valid=None, chunk_size=128, count_paths=True, revolute_joints=None,
                  node_valid=None, stats=None, step_limits=None, cancel_check=None,
                  node_rejection_group=None):
    """Minimize summed weighted joint distances over all adjacent-layer edges.

    No random endpoints or materialized graph. Memory is bounded by a block
    of pair costs and one predecessor per candidate. periodic applies only
    to explicitly continuous joints; bounded joints use actual angle deltas.
    edge_valid(layer_index, previous_index, next_index) can reject transitions;
    previous_index=-1 denotes the initial configuration.
    count_paths=False skips the potentially huge integer path-count calculation.
    revolute_joints identifies angular axes eligible for full-turn indexing;
    it does not make bounded joints periodic or remove their limits.
    node_valid(i,j) enables exact graph-first configuration validation. Rejected
    nodes are removed and the graph re-solved; count_paths must be False. Returned
    indices always refer to the original input layers. step_limits optionally
    supplies an N x joints array of additional per-transition bounds.
    node_rejection_group(i,j) may supply additional verified invalid original
    indices in layer i, for example after completing that layer's collision
    checks. It never merges states; limits, costs and ties remain exact.
    """
    if node_valid is not None:
        if count_paths:
            raise ValueError('node_valid requires count_paths=False; unchecked alternatives cannot be counted')
        return _validate_nodes(layers, node_valid, stats, node_rejection_group=node_rejection_group,
            start=start, weights=weights,
            periodic=periodic, max_step=max_step, edge_valid=edge_valid, chunk_size=chunk_size,
            revolute_joints=revolute_joints, step_limits=step_limits, cancel_check=cancel_check)
    if chunk_size < 1:
        raise ValueError('chunk_size must be positive')
    if not layers:
        return GraphResult([], [], 0.0)
    if any(len(layer) == 0 for layer in layers):
        return GraphResult([], [], float('inf'), failure_layer=next(i for i,l in enumerate(layers) if not len(l)))
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
    global_limit = limit.copy()
    per_layer = None if step_limits is None else np.asarray(step_limits, dtype=float)
    if per_layer is not None and (per_layer.shape != (len(arrays), n) or
                                 np.any(np.isnan(per_layer)) or np.any(per_layer < 0)):
        raise ValueError('step_limits must be a nonnegative N x joints array')
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
        if cancel_check is not None:
            cancel_check()
        limit = global_limit if per_layer is None else np.minimum(global_limit, per_layer[i])
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
                    return GraphResult([], [], float('inf'), failure_layer=i)
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
                values = costs[indices] + np.linalg.norm(delta*w, axis=1)
                if edge_valid is not None and not count_paths:
                    # Cost order is exact for a layered DAG: the first valid
                    # predecessor wins. Stable sorting preserves original ties.
                    for chosen in np.argsort(values, kind='stable'):
                        a = int(indices[chosen])
                        if edge_valid(i, a if i > 0 else -1, b):
                            next_costs[b], pred[b] = values[chosen], a
                            break
                    continue
                if edge_valid is not None:
                    keep = np.array([edge_valid(i, int(a) if i > 0 else -1, b) for a in indices], dtype=bool)
                    indices, values = indices[keep], values[keep]
                if not len(indices):
                    continue
                chosen = int(values.argmin())
                next_costs[b], pred[b] = values[chosen], indices[chosen]
                if count_paths:
                    next_counts[b] = sum(counts[a] for a in indices)
            if not np.isfinite(next_costs).any():
                event('graph.disconnected', layer=i)
                return GraphResult([], [], float('inf'), failure_layer=i,
                                   reachable_indices=tuple(int(a) for a in active))
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
            if edge_valid is not None and not count_paths:
                for b in range(len(block)):
                    for a in np.argsort(values[:, b], kind='stable'):
                        if not np.isfinite(values[a,b]):
                            break
                        if edge_valid(i, int(a) if i > 0 else -1, int(offset+b)):
                            next_costs[offset+b], pred[offset+b] = values[a,b], a
                            break
                continue
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
            return GraphResult([], [], float('inf'), failure_layer=i,
                               reachable_indices=tuple(int(a) for a in active))
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


def _validate_nodes(layers, check, stats, node_rejection_group=None, **options):
    """Lazy node rejection around the same exact solver, with original indices."""
    mappings = [list(range(len(layer))) for layer in layers]
    accepted = set()
    stats = {} if stats is None else stats
    stats.update(graph_solves=0, graph_seconds=0., node_checks=0, node_rejections=0, additional_rejections=0,
                 node_check_seconds=0., original_nodes=sum(map(len,layers)))
    original_edge = options.pop('edge_valid')
    while True:
        working = [[layers[i][j] for j in ids] for i,ids in enumerate(mappings)]
        edge = None if original_edge is None else lambda i,a,b: original_edge(
            i, -1 if a < 0 else mappings[i-1][a], mappings[i][b])
        started = perf_counter()
        result = shortest_path(working, edge_valid=edge, count_paths=False, **options)
        stats['graph_seconds'] += perf_counter()-started
        stats['graph_solves'] += 1
        if not result.configurations:
            if result.failure_layer is not None and result.failure_layer > 0:
                result.reachable_indices = tuple(mappings[result.failure_layer-1][j] for j in result.reachable_indices)
            break
        selected = [mappings[i][j] for i,j in enumerate(result.indices)]
        rejected = []
        for i,j in enumerate(selected):
            if options['cancel_check'] is not None:
                options['cancel_check']()
            if (i,j) in accepted:
                continue
            started = perf_counter()
            valid = check(i,j)
            stats['node_check_seconds'] += perf_counter()-started
            stats['node_checks'] += 1
            if valid:
                accepted.add((i,j))
            else:
                rejected.append((i,j))
                stats['node_rejections'] += 1
        event('graph.configuration_validation', iteration=stats['graph_solves'],
              rejected_nodes=rejected, optimistic_cost=result.cost)
        if not rejected:
            result.indices = selected
            break
        for i,j in rejected:
            group = {j} if node_rejection_group is None else set(node_rejection_group(i,j)) | {j}
            if any(k<0 or k>=len(layers[i]) for k in group):
                raise ValueError('node_rejection_group returned invalid original indices')
            removed = set(mappings[i]) & group
            stats['additional_rejections'] += len(removed)-1
            mappings[i] = [k for k in mappings[i] if k not in group]
    for name,value in stats.items():
        metric('graph.'+name, value, 's' if name.endswith('_seconds') else 'count')
    return result

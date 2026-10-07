"""Shared shoulder/elbow/wrist branch constraints for sampled arm paths."""
from collections import Counter
import math
import numpy as np
from .graph import GraphResult, shortest_path


class ConfigurationBranchCheck:
    def __init__(self, solver, layers, start=None, periodic=None):
        branch_of = getattr(solver, 'configuration_branch', None)
        self.applied = callable(branch_of)
        self.layers, self.start = layers, start
        self.periodic = periodic
        self.branches = [[branch_of(q) for q in rows] for rows in layers] if self.applied else None
        self.start_branch = branch_of(start) if self.applied and start is not None else None
        self.rejections = Counter()

    def node_valid(self, i, j):
        if self.applied and self.branches[i][j] is None:
            self.rejections['ambiguous_configuration_branch'] += 1
            return False
        return True

    def edge_valid(self, i, a, b):
        before = self.start_branch if a < 0 else self.branches[i-1][a]
        after = self.branches[i][b]
        if before is None or after is None:
            self.rejections['ambiguous_configuration_branch'] += 1
            return False
        changed = [name for name, x, y in zip(('shoulder', 'elbow', 'wrist'), before, after) if x != y]
        if changed:
            self.rejections.update(name+'_branch_change' for name in changed)
            return False
        q0 = np.asarray(self.start if a < 0 else self.layers[i-1][a])
        delta = np.asarray(self.layers[i][b])-q0
        if self.periodic is not None:
            periodic = np.asarray(self.periodic, dtype=bool)
            delta[periodic] = (delta[periodic]+math.pi) % (2*math.pi)-math.pi
        if any(abs(delta[j]) >= math.pi for j in (2, 4)):
            self.rejections['branch_boundary_crossing'] += 1
            return False
        return True


def shortest_branch_path(layers, *, solver, start=None, **options):
    """Apply branch constraints while preserving original graph indices/callbacks.

    Remove ambiguous nodes before solving so exact path counts also exclude them.
    Collision validation remains lazy when the caller supplies node callbacks.
    """
    check = ConfigurationBranchCheck(solver, layers, start, options.get('periodic'))
    initial_failure = None
    if not check.applied:
        solved = shortest_path(layers, start=start, **options)
    elif start is not None and check.start_branch is None:
        check.rejections['ambiguous_configuration_branch'] += 1
        initial_failure = 'Starting configuration lies on an ambiguous shoulder/elbow/wrist branch boundary'
        solved = GraphResult([], [], float('inf'), failure_layer=0)
    else:
        indices = [[j for j in range(len(rows)) if check.node_valid(i, j)] for i, rows in enumerate(layers)]
        edge = options.pop('edge_valid', None)
        node = options.pop('node_valid', None)
        group = options.pop('node_rejection_group', None)
        def edge_valid(i, a, b):
            a, b = indices[i-1][a] if a >= 0 else -1, indices[i][b]
            return check.edge_valid(i, a, b) and (edge is None or edge(i, a, b))
        def rejection_group(i, j):
            rejected = set(group(i, indices[i][j]))
            return [k for k, original in enumerate(indices[i]) if original in rejected]
        solved = shortest_path([[rows[j] for j in ids] for rows, ids in zip(layers, indices)],
            start=start, edge_valid=edge_valid,
            node_valid=(lambda i, j: node(i, indices[i][j])) if node else None,
            node_rejection_group=rejection_group if group else None, **options)
        solved.indices = [indices[i][j] for i, j in enumerate(solved.indices)]
        if solved.failure_layer is not None and solved.failure_layer > 0:
            solved.reachable_indices = tuple(indices[solved.failure_layer-1][j] for j in solved.reachable_indices)
    return solved, dict(configuration_branch_check_applied=check.applied,
        selected_configuration_branch=list(check.branches[0][solved.indices[0]])
            if solved.configurations and check.applied else None,
        edge_rejection_reasons=dict(check.rejections), initial_state_failure=initial_failure)

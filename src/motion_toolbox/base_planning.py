"""Stationary placement and coupled base/arm trajectory search."""
from dataclasses import dataclass
import math
import numpy as np
from .geometry import Plane, as_plane
from .planning import candidates, calculate_partial_trajectory
from .graph import shortest_path


def grid_bases(x_values, y_values, yaw_values=(0.0,), z=0.0):
    """Explicit bounded search domain; yaw values are radians."""
    return [Plane((x, y, z), (math.cos(a), math.sin(a), 0), (-math.sin(a), math.cos(a), 0))
            for x in x_values for y in y_values for a in yaw_values]


def bases_around_targets(targets, distances, bearings, yaw_offsets=(0.0,), z=0.0):
    """Generate upright footprint candidates facing each TCP in the XY plane.

    IK and collision checking select feasibility; this only supplies a search
    domain. Distances are metres, bearings and yaw offsets are radians.
    """
    layers = []
    for target in targets:
        p = as_plane(target).origin
        layer = []
        for distance in distances:
            if distance <= 0:
                raise ValueError('Positive candidate distance required')
            for bearing in bearings:
                x, y = p[:2] + distance*np.array([math.cos(bearing), math.sin(bearing)])
                for offset in yaw_offsets:
                    yaw = bearing + math.pi + offset
                    layer.append(Plane((x, y, z), (math.cos(yaw), math.sin(yaw), 0), (-math.sin(yaw), math.cos(yaw), 0)))
        layers.append(layer)
    return layers


@dataclass
class BasePlan:
    base_planes: list
    configurations: list
    cost: float
    candidate_counts: list
    diagnostics: list

    @property
    def base_plane(self):
        """Single stationary base; moving results use base_planes."""
        if len(self.base_planes) != 1:
            raise ValueError('Result does not contain one stationary base')
        return self.base_planes[0]


def evaluate_base_locations(targets, base_candidates, *, ik_solver, collision=None,
                            offsets=(0.0,), joint_ranges=None):
    """Candidate-only fitness for Grasshopper sliders; no graph construction.

    Counts every target (never silently subsamples). A high count is a useful
    diagnostic, not proof that the layers have a connected motion path.
    """
    results = []
    for base in base_candidates:
        counts, before = [], []
        for target in targets:
            q, raw, _ = candidates(target, base, ik_solver, offsets, collision, joint_ranges)
            counts.append(len(q))
            before.append(raw)
        results.append(dict(base_plane=as_plane(base), solution_counts=counts,
            solution_counts_before_collision=before,
            unreachable_points=[i for i, n in enumerate(counts) if not n],
            reachable_targets=sum(n > 0 for n in counts), total_solutions=sum(counts),
            fitness=(-sum(n == 0 for n in counts) if any(n == 0 for n in counts) else 1.0+sum(counts)/100000.0)))
    return results


def find_stationary_base(targets, base_candidates, current_pose, *, ik_solver,
                         collision=None, transition_check=None, **options):
    """Return one base that supports a complete connected arm path.

    Optimize joint travel over the finite base candidate set. No feasible
    complete path returns empty base_planes/configurations and infinite cost.
    """
    if not targets:
        raise ValueError('At least one target required for placement search')
    best = BasePlan([], [], float('inf'), [], [])
    for base in base_candidates:
        base = as_plane(base)
        evaluation = calculate_partial_trajectory(current_pose, targets, base_planes=[base],
            ik_solver=ik_solver, collision=collision, dont_build_graph=True, **options)
        layers = evaluation['ik_solutions_per_node']
        edge = None
        if transition_check is not None:
            def edge(i, a, b):
                q0 = current_pose if i == 0 else layers[i-1][a]
                return transition_check(q0, base, layers[i][b], base)
        solved = shortest_path(layers, start=current_pose,
            max_step=options.get('max_joint_step', 2.5), periodic=options.get('periodic'),
            weights=options.get('weights'), edge_valid=edge)
        best.diagnostics.append(dict(base_plane=base, cost=solved.cost,
                                     unreachable_points=evaluation['unreachable_points']))
        if solved.configurations and solved.cost < best.cost:
            best.base_planes, best.configurations, best.cost = [base], solved.configurations, solved.cost
            best.candidate_counts = [len(layer) for layer in layers]
    return best


def plan_mobile_base(targets, base_candidates_per_target, *, ik_solver, current_pose=None,
        start_base=None, collision=None, transition_check=None, offsets=(0.0,), joint_ranges=None,
        max_base_step=0.25, max_yaw_step=0.25, max_joint_step=2.5,
        base_weight=1.0, yaw_weight=1.0, joint_weights=None, periodic=None,
        time_intervals=None, max_base_speed=None, max_yaw_speed=None, max_joint_speed=None):
    """Jointly optimize arm configuration and holonomic base pose per TCP target.

    Exact over the supplied discretized states, with weighted Euclidean step
    cost. Caller-supplied time_intervals include the initial-to-first interval
    when start_base is supplied; otherwise they cover target-to-target edges.
    transition_check(q0, b0, q1, b1) optionally rejects swept collisions or
    steering constraints. No candidate pruning that could discard an optimum.
    """
    if len(targets) != len(base_candidates_per_target) or not targets:
        raise ValueError('Provide a nonempty candidate layer for every target')
    if (current_pose is None) != (start_base is None):
        raise ValueError('Specify both current_pose and start_base, or neither')
    states, numeric = [], []
    arm_dimension = None
    for target, bases in zip(targets, base_candidates_per_target):
        states_at_target, numeric_at_target = [], []
        for base in bases:
            base = as_plane(base)
            if not np.allclose(base.zaxis, (0, 0, 1)):
                raise ValueError('Mobile bases must be upright')
            qs, _, _ = candidates(target, base, ik_solver, offsets, collision, joint_ranges)
            for q in qs:
                arm_dimension = len(q) if arm_dimension is None else arm_dimension
                if len(q) != arm_dimension:
                    raise ValueError('IK dimensions must match')
                yaw = math.atan2(base.xaxis[1], base.xaxis[0])
                states_at_target.append((q, base))
                numeric_at_target.append(list(base.origin)+[yaw]+q)
        states.append(states_at_target)
        numeric.append(numeric_at_target)
    counts = [len(layer) for layer in states]
    if not all(counts):
        return BasePlan([], [], float('inf'), counts, [])
    n = arm_dimension
    arm_periodic = [False]*n if periodic is None else list(periodic)
    weights = [base_weight]*3+[yaw_weight]+(list(joint_weights) if joint_weights is not None else [1]*n)
    start = None
    if start_base is not None:
        start_base = as_plane(start_base)
        if not np.allclose(start_base.zaxis, (0, 0, 1)):
            raise ValueError('Start base must be upright')
        start = list(start_base.origin)+[math.atan2(start_base.xaxis[1], start_base.xaxis[0])]+list(current_pose)
    edge_count = len(targets) if start is not None else len(targets)-1
    if time_intervals is not None:
        if len(time_intervals) != edge_count or any(t <= 0 or not math.isfinite(t) for t in time_intervals):
            raise ValueError('One positive duration per edge required')
    elif any(v is not None for v in (max_base_speed, max_yaw_speed, max_joint_speed)):
        raise ValueError('Speed constraints require time_intervals')
    if any(v is not None and (v < 0 or not math.isfinite(v)) for v in (max_base_step, max_yaw_step, max_base_speed, max_yaw_speed)):
        raise ValueError('Base limits must be finite and nonnegative')

    def edge(i, a, b):
        q0, b0 = (current_pose, start_base) if i == 0 else states[i-1][a]
        q1, b1 = states[i][b]
        distance = np.linalg.norm(b1.origin-b0.origin)
        y0, y1 = math.atan2(b0.xaxis[1], b0.xaxis[0]), math.atan2(b1.xaxis[1], b1.xaxis[0])
        yaw = abs((y1-y0+math.pi) % (2*math.pi)-math.pi)
        if (max_base_step is not None and distance > max_base_step) or (max_yaw_step is not None and yaw > max_yaw_step):
            return False
        if time_intervals is not None:
            dt = time_intervals[i if start is not None else i-1]
            if max_base_speed is not None and distance > max_base_speed*dt:
                return False
            if max_yaw_speed is not None and yaw > max_yaw_speed*dt:
                return False
            delta = np.array(q1)-q0
            delta[arm_periodic] = (delta[arm_periodic]+math.pi) % (2*math.pi)-math.pi
            if max_joint_speed is not None and np.any(np.abs(delta) > np.asarray(max_joint_speed)*dt):
                return False
        return transition_check is None or transition_check(q0, b0, q1, b1)

    limits = [np.inf]*4+np.broadcast_to(np.inf if max_joint_step is None else max_joint_step, (n,)).tolist()
    solved = shortest_path(numeric, start=start, weights=weights, periodic=[False]*3+[True]+arm_periodic,
                           max_step=limits, edge_valid=edge)
    chosen_bases = [states[i][j][1] for i, j in enumerate(solved.indices)]
    return BasePlan(chosen_bases, [q[4:] for q in solved.configurations], solved.cost, counts, [])

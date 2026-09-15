"""Stationary placement and coupled base/arm trajectory search."""
from motion_toolbox.recording import recorded
from dataclasses import dataclass
import math
from time import perf_counter
import numpy as np
from .geometry import Plane, as_plane
from .planning import candidates, calculate_partial_trajectory, rotation_offsets
from .graph import shortest_path


@recorded
def grid_bases(x_values, y_values, yaw_values=(0.0,), z=0.0):
    """Explicit bounded search domain; yaw values are radians."""
    return [Plane((x, y, z), (math.cos(a), math.sin(a), 0), (-math.sin(a), math.cos(a), 0))
            for x in x_values for y in y_values for a in yaw_values]


@recorded
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


@recorded
def stationary_base_guesses(targets, *, arm_in_base=None, distance=1.0,
                           margin=1.5, yaw_steps=4, z=0.0):
    """Geometry-only seeds in metres, inspired by distance/offset curve placement.

    Use a geometric median at arm height, a least-squares equidistant center,
    and offsets to both sides of the target cloud. Convert desired arm origins
    to footprint origins using the mounting offset. All seeds stay within the
    target XY bounds plus margin. These are proposals, not reach/collision tests.
    """
    targets = [as_plane(t) for t in targets]
    if not targets:
        raise ValueError('At least one target required')
    if not all(math.isfinite(v) for v in (distance, margin, z)) or distance <= 0 or margin < 0:
        raise ValueError('Positive finite guess distance and nonnegative margin required')
    if int(yaw_steps) != yaw_steps or yaw_steps < 1:
        raise ValueError('yaw_steps must be a positive integer')
    mount = as_plane(arm_in_base) if arm_in_base is not None else Plane.world_xy()
    points = np.array([t.origin for t in targets])
    xy = points[:, :2]
    height = z + mount.origin[2]
    center = xy.mean(axis=0)
    median = center.copy()
    for _ in range(200):
        lengths = np.sqrt(np.sum((xy-median)**2, axis=1)+(points[:, 2]-height)**2)
        weights = 1 / np.maximum(lengths, 1e-9)
        updated = np.average(xy, axis=0, weights=weights)
        movement = np.linalg.norm(updated-median)
        median = updated
        if movement < 1e-7:
            break
    centers = [median, (xy.min(axis=0)+xy.max(axis=0))/2]
    # Center coordinates before fitting to avoid sensitivity to world offsets.
    local = xy-center
    matrix = np.column_stack((2*local, np.ones(len(points))))
    rhs = np.sum(local**2, axis=1)+(points[:, 2]-height)**2
    fitted, _, rank, singular = np.linalg.lstsq(matrix, rhs, rcond=None)
    if rank == 3 and singular[-1] > singular[0]*1e-8:
        centers.append(center+fitted[:2])
    # PCA handles a straight wall even when target normals point vertically.
    _, directions = np.linalg.eigh(local.T @ local)
    sideways = directions[:, 0]
    normals = [sideways]
    normal = np.mean([t.zaxis[:2] for t in targets], axis=0)
    if np.linalg.norm(normal) > 1e-8:
        normals.append(normal/np.linalg.norm(normal))
    for normal in normals:
        for sign in (-1, 1):
            centers.append(median+sign*distance*normal)
    lower, upper = xy.min(axis=0)-margin, xy.max(axis=0)+margin
    result, seen = [], set()
    for arm_xy in centers:
        toward = center-arm_xy
        facing = math.atan2(toward[1], toward[0]) if np.linalg.norm(toward) > 1e-8 else 0.0
        for yaw in facing + np.arange(int(yaw_steps))*2*math.pi/yaw_steps:
            c, s = math.cos(yaw), math.sin(yaw)
            rotated_offset = np.array([c*mount.origin[0]-s*mount.origin[1],
                                       s*mount.origin[0]+c*mount.origin[1]])
            footprint = arm_xy-rotated_offset
            if np.any(footprint < lower-1e-9) or np.any(footprint > upper+1e-9):
                continue
            base = Plane((*footprint, z), (c, s, 0), (-s, c, 0))
            key = tuple(np.round(base.matrix.ravel(), 9))
            if key not in seen:
                seen.add(key)
                distances = np.sqrt(np.sum((xy-arm_xy)**2, axis=1)+(points[:, 2]-height)**2)
                result.append((float(distances.max()), float(distances.std()), base))
    result.sort(key=lambda item: item[:2])
    return [item[2] for item in result]


@recorded
def stationary_base_candidates(targets, *, margin=1.5, spacing=0.5,
                               yaw_steps=4, z=0.0, initial_guesses=()):
    """Upright footprint grid around the XY bounds of all targets, in metres.

    This is a finite placement domain, not a reachability approximation. Every
    placement still needs IK and optional collision checks. Initial guesses are
    evaluated first, with duplicate grid placements removed. Refine spacing/yaw
    or enlarge margin if the supplied domain misses a feasible placement.
    """
    targets = [as_plane(t) for t in targets]
    if not targets:
        raise ValueError('At least one target required')
    if not all(math.isfinite(v) for v in (margin, spacing, z)) or margin < 0 or spacing <= 0:
        raise ValueError('Finite nonnegative margin and positive spacing required')
    if int(yaw_steps) != yaw_steps or yaw_steps < 1:
        raise ValueError('yaw_steps must be a positive integer')
    xy = np.array([t.origin[:2] for t in targets])
    lower, upper = xy.min(axis=0)-margin, xy.max(axis=0)+margin
    axes = [np.linspace(a, b, int(math.ceil((b-a)/spacing))+1) for a,b in zip(lower, upper)]
    grid = grid_bases(*axes, yaw_values=np.arange(int(yaw_steps))*2*math.pi/yaw_steps, z=z)
    result, seen = [], set()
    for base in list(initial_guesses) + grid:
        base = as_plane(base)
        key = tuple(np.round(base.matrix.ravel(), 9))
        if key not in seen:
            seen.add(key)
            result.append(base)
    return result


@dataclass
class BasePlan:
    base_planes: list
    configurations: list
    cost: float
    candidate_counts: list
    diagnostics: list
    path_count: int = 0
    standoff: float = 0.0
    max_target_distance: float = 0.0
    ik_option_count: int = 0
    path_search_count: int = 0
    validation_attempts: int = 0
    base_collision_checks: int = 0
    heuristic_plane: object = None

    @property
    def base_plane(self):
        """Single stationary base; moving results use base_planes."""
        if len(self.base_planes) != 1:
            raise ValueError('Result does not contain one stationary base')
        return self.base_planes[0]


@recorded
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


@recorded
def find_stationary_base(targets, base_candidates, current_pose=None, *, ik_solver,
                         collision=None, transition_check=None, objective='joint_travel',
                         placement_region=None, build_path=True, base_collision=None,
                         max_validation_attempts=3, **options):
    """Return one base that supports a complete connected arm path.

    objective='max_paths' maximizes the number of complete valid configuration
    sequences in the sampled graph, breaking ties by minimum joint travel.
    With placement_region, enforce its side/distance rules before IK and prefer
    greater minimum wall standoff before travel when path counts tie.
    objective='ik_options' ranks collision-free target configuration combinations,
    then standoff, without building graphs. It builds at most one joint path for
    the winner (unless build_path=False). No transition collision checks are used
    in this mode; returned base_planes can be nonempty even if the path fails.
    The default 'joint_travel' minimizes travel over the finite candidate set. No feasible
    complete path returns empty base_planes/configurations and infinite cost.
    Omit current_pose to optimize only target-to-target travel, without an
    assumed initial configuration or approach transition.
    """
    if not targets:
        raise ValueError('At least one target required for placement search')
    if objective == 'heuristic':
        if placement_region is None:
            raise ValueError('Heuristic placement requires a placement_region')
        if transition_check is not None or options.get('edge_valid') is not None:
            raise ValueError('Heuristic placement checks configurations only')
        return _find_stationary_base_heuristic(targets, base_candidates, current_pose,
            ik_solver, collision, placement_region, build_path, options,
            base_collision, max_validation_attempts)
    if objective == 'ik_options':
        if transition_check is not None or options.get('edge_valid') is not None:
            raise ValueError('ik_options mode checks collisions only at target configurations; omit transition callbacks')
        return _find_stationary_base_by_options(targets, base_candidates, current_pose,
            ik_solver, collision, placement_region, build_path, options)
    if objective not in ('joint_travel', 'max_paths'):
        raise ValueError("objective must be 'joint_travel', 'max_paths', 'ik_options' or 'heuristic'")
    best = BasePlan([], [], float('inf'), [], [])
    best_rank = None
    for base in base_candidates:
        base = as_plane(base)
        geometry = placement_region.metrics(base) if placement_region is not None else {}
        if geometry and not geometry['geometry_valid']:
            failures = sorted(set(geometry['wrong_side_points'] + geometry['too_far_points']))
            reasons = []
            if geometry['wrong_side_points']:
                reasons.append('wrong_side')
            if geometry['too_far_points']:
                reasons.append('beyond_reach_limit')
            best.diagnostics.append(dict(base_plane=base, cost=float('inf'),
                unreachable_points=failures, solution_counts_before_filters=[],
                solution_counts=[], reachable_targets=0, path_count=0,
                reason='+'.join(reasons), ik_checked=False, **geometry))
            continue
        evaluation = calculate_partial_trajectory(current_pose, targets, base_planes=[base],
            ik_solver=ik_solver, collision=collision, dont_build_graph=True, **options)
        layers = evaluation['ik_solutions_per_node']
        edge = None
        edge_callback = options.get('edge_valid')
        if transition_check is not None or edge_callback is not None:
            def edge(i, a, b):
                if edge_callback is not None and not edge_callback(i, a, b):
                    return False
                q0 = current_pose if i == 0 else layers[i-1][a]
                return transition_check is None or transition_check(q0, base, layers[i][b], base)
        solved = shortest_path(layers, start=current_pose,
            max_step=options.get('max_joint_step', 2.5), periodic=options.get('periodic'),
            weights=options.get('weights'), edge_valid=edge,
            revolute_joints=getattr(ik_solver, 'revolute_joints', None))
        counts = [len(layer) for layer in layers]
        raw = evaluation['solution_counts_before_collision']
        reason = ('complete' if solved.configurations else
                  'no_ik' if any(n == 0 for n in raw) else
                  'joint_limits_or_collision' if not all(counts) else 'blocked_transitions')
        best.diagnostics.append(dict(base_plane=base, cost=solved.cost,
            unreachable_points=evaluation['unreachable_points'],
            solution_counts_before_filters=raw, solution_counts=counts,
            reachable_targets=sum(n > 0 for n in counts), path_count=solved.path_count,
            reason=reason, ik_checked=True, **geometry))
        standoff = geometry.get('standoff', 0.0)
        rank = (-solved.path_count, -standoff, solved.cost) if objective == 'max_paths' else (solved.cost,)
        if solved.configurations and (best_rank is None or rank < best_rank):
            best_rank = rank
            best.base_planes, best.configurations, best.cost = [base], solved.configurations, solved.cost
            best.candidate_counts = counts
            best.path_count = solved.path_count
            best.standoff = standoff
            best.max_target_distance = geometry.get('max_target_distance', 0.0)
    return best


def _find_stationary_base_heuristic(targets, bases, current_pose, solver, collision,
                                   region, build_path, options, base_collision, attempts):
    """Rank geometry only; check base body, then validate a bounded shortlist.

    Stop at the first fully reachable position. Collision failures try other
    headings; IK reach failures move inward. Retry failed targets first.
    No IK counts are computed for ranking and at most one path is constructed.
    """
    if int(attempts) != attempts or attempts < 1:
        raise ValueError('max_validation_attempts must be a positive integer')
    ranked = []
    diagnostics = []
    for base in bases:
        base = as_plane(base)
        metrics = region.metrics(base)
        if metrics['geometry_valid']:
            ranked.append((base, metrics))
        else:
            diagnostics.append(dict(base_plane=base, reason='wrong_side_or_distance',
                ik_checked=False, targets_checked=0, reachable_targets=0, solution_counts=[],
                path_checked=False, **metrics))
    ranked.sort(key=lambda item: -item[1]['standoff'])
    result = BasePlan([], [], float('inf'), [], diagnostics)
    tried = []
    previous_standoff = None
    previous_reason = None
    priority_targets = []
    checks = 0
    guess = None
    while ranked and len(tried) < attempts:
        # A reach-boundary point can fail in 3D even though its XY radius fits.
        # Move meaningfully inward after failure, not sideways along that same
        # boundary or through every heading at the same overextended location.
        index = next((i for i, (_, m) in enumerate(ranked)
                      if (previous_standoff is None or m['standoff'] <= .75*previous_standoff)
                      and all(np.linalg.norm(np.array(m['arm_origin'])[:2]-p) > .1 for p in tried)),
                     next((i for i, (_, m) in enumerate(ranked)
                           if all(np.linalg.norm(np.array(m['arm_origin'])[:2]-p) > .1 for p in tried)), 0))
        if previous_reason == 'collision':
            # Chassis/GPS/tool interference depends on heading. Do not discard
            # headings at the same shoulder position or force the arm inward.
            index = next((i for i, (_, m) in enumerate(ranked)
                          if np.linalg.norm(np.array(m['arm_origin'])[:2]-tried[-1]) < 1e-6), 0)
        base, metrics = ranked.pop(index)
        if base_collision is not None:
            checks += 1
            if not base_collision(base):
                diagnostics.append(dict(base_plane=base, reason='base_environment_collision',
                    base_collision_detail=getattr(getattr(base_collision, '__self__', None), 'last_failure', None),
                    ik_checked=False, targets_checked=0, reachable_targets=0, solution_counts=[],
                    path_checked=False, **metrics))
                continue
        if guess is None:
            guess = base
        tried.append(np.array(metrics['arm_origin'])[:2])
        previous_standoff = metrics['standoff']
        candidate = _find_stationary_base_by_options(targets, [base], current_pose,
            solver, collision, region, build_path, dict(options, _priority_targets=priority_targets))
        diagnostics.extend(candidate.diagnostics)
        if candidate.base_planes:
            result = candidate
            break
        previous_reason = candidate.diagnostics[-1]['reason']
        failed = candidate.diagnostics[-1].get('failed_target_details')
        if failed is not None:
            index = failed['target_index']
            priority_targets = [index] + [i for i in priority_targets if i != index]
    result.diagnostics = diagnostics
    result.validation_attempts = len(tried)
    result.base_collision_checks = checks
    result.heuristic_plane = guess
    return result


def _find_stationary_base_by_options(targets, bases, current_pose, solver, collision,
                                     region, build_path, options):
    """Configuration-only ranking with early rejection and one winner's graph."""
    offsets = rotation_offsets(options.get('rotation_mode', False),
        options.get('rotation_angle_deg', 5), options.get('rotation_steps', 35),
        options.get('angle_cw_deg', 0), options.get('angle_ccw_deg', 0))
    best = BasePlan([], [], float('inf'), [], [])
    best_layers, best_rank, selected = None, None, None
    for base in bases:
        base = as_plane(base)
        geometry = region.metrics(base) if region is not None else {}
        diagnostic = dict(base_plane=base, cost=float('inf'), unreachable_points=[],
            solution_counts_before_filters=[], solution_counts=[], reachable_targets=0,
            path_count=0, ik_option_count=0, ik_checked=False, targets_checked=0,
            path_checked=False, reason='', **geometry)
        best.diagnostics.append(diagnostic)
        if geometry and not geometry['geometry_valid']:
            reasons = []
            if geometry['wrong_side_points']:
                reasons.append('wrong_side')
            if geometry['too_far_points']:
                reasons.append('beyond_reach_limit')
            diagnostic['reason'] = '+'.join(reasons)
            diagnostic['unreachable_points'] = sorted(set(geometry['wrong_side_points']+geometry['too_far_points']))
            continue
        layers, raw = {}, {}
        diagnostic['timings'] = dict(ik_seconds=0.0, joint_expansion_seconds=0.0, collision_seconds=0.0)
        priority = options.get('_priority_targets', [])
        order = list(dict.fromkeys(list(priority) + list(range(len(targets)))))
        for i in order:
            target = targets[i]
            stats = {}
            qs, before, _ = candidates(target, base, solver, offsets, collision, options.get('joint_ranges'), stats=stats)
            for name in diagnostic['timings']:
                diagnostic['timings'][name] += stats[name]
            layers[i] = qs
            raw[i] = before
            if not qs:
                diagnostic['unreachable_points'] = [i]
                diagnostic['reason'] = ('no_ik' if before == 0 else
                    'joint_limits' if stats['within_joint_limits'] == 0 else 'collision')
                diagnostic['failed_target_details'] = dict(target_index=i, **stats)
                break  # This base cannot cover all targets; no reason to test the rest.
        checked = sorted(layers)
        layers = [layers[i] for i in checked]
        raw = [raw[i] for i in checked]
        counts = [len(layer) for layer in layers]
        diagnostic.update(ik_checked=True, targets_checked=len(layers),
            checked_target_indices=checked,
            solution_counts=counts, solution_counts_before_filters=raw,
            reachable_targets=sum(n > 0 for n in counts))
        if diagnostic['unreachable_points']:
            continue
        combinations = math.prod(counts)
        standoff = geometry.get('standoff', 0.0)
        diagnostic.update(reason='all_targets_reachable', ik_option_count=combinations)
        rank = (-combinations, -standoff)
        if best_rank is None or rank < best_rank:
            best_rank, best_layers, selected = rank, layers, diagnostic
            best.base_planes, best.candidate_counts = [base], counts
            best.ik_option_count, best.standoff = combinations, standoff
            best.max_target_distance = geometry.get('max_target_distance', 0.0)
    if best_layers is not None and build_path:
        path_started = perf_counter()
        solved = shortest_path(best_layers, start=current_pose,
            max_step=options.get('max_joint_step', 2.5), periodic=options.get('periodic'),
            weights=options.get('weights'), count_paths=options.get('count_paths', True),
            revolute_joints=getattr(solver, 'revolute_joints', None))
        selected['timings']['path_seconds'] = perf_counter()-path_started
        best.configurations, best.cost, best.path_count = solved.configurations, solved.cost, solved.path_count
        best.path_search_count = 1
        selected.update(path_checked=True, path_count=solved.path_count, cost=solved.cost,
            reason='complete' if solved.configurations else 'joint_step_disconnected')
    return best


@recorded
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
    if max_joint_speed is not None:
        speeds = np.asarray(max_joint_speed, dtype=float)
        if speeds.ndim > 1 or (speeds.ndim == 1 and speeds.shape != (n,)) or not np.isfinite(speeds).all() or np.any(speeds < 0):
            raise ValueError('Joint speeds must be nonnegative finite scalars or one value per joint')
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

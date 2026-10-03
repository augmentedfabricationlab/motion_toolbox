"""Stationary base placement and arm trajectory search."""
from motion_toolbox.recording import recorded, event
from dataclasses import dataclass, field
import math
from time import perf_counter
from typing import Optional
import numpy as np
from .geometry import Plane, as_plane
from .planning import candidates, calculate_partial_trajectory, rotation_offsets
from .graph import shortest_path
from .execution import high_qos, check_cancel


@recorded
def grid_bases(x_values, y_values, yaw_values=(0.0,), z=0.0):
    """Explicit bounded search domain; yaw values are radians."""
    return [Plane((x, y, z), (math.cos(a), math.sin(a), 0), (-math.sin(a), math.cos(a), 0))
            for x in x_values for y in y_values for a in yaw_values]




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
    ik_option_count: Optional[int] = 0
    path_search_count: int = 0
    validation_attempts: int = 0
    base_collision_checks: int = 0
    heuristic_plane: object = None
    ik_solutions_per_node: list = field(default_factory=list)
    target_diagnostics: list = field(default_factory=list)
    counts_complete: bool = True
    selected_target_planes: list = field(default_factory=list)
    selected_tcp_rotations: list = field(default_factory=list)
    initial_state_failure: Optional[str] = None
    disconnected_detail: Optional[dict] = None

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
@high_qos
def find_stationary_base(targets, base_candidates, current_pose=None, *, ik_solver,
                         collision=None, transition_check=None, objective='joint_travel',
                         placement_region=None, build_path=True, base_collision=None,
                         max_validation_attempts=3, fast_validation=True, **options):
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
    Heuristic fast_validation skips unused collision alternatives and leaves
    counts incomplete. Explicit count_paths=True forces exhaustive validation.
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
            base_collision, max_validation_attempts,
            fast_validation and not options.get('count_paths', False))
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
                                   region, build_path, options, base_collision, attempts, fast=False):
    """Rank geometry only; check base body, then validate a bounded shortlist.

    Stop at the first fully reachable position. Collision failures try other
    headings; IK reach failures move inward. Retry failed targets first.
    No IK counts are computed for ranking and at most one path is constructed.
    """
    if int(attempts) != attempts or attempts < 1:
        raise ValueError('max_validation_attempts must be a positive integer')
    if current_pose is not None:
        from .planning import _in_ranges, normalize_joint_ranges
        if not np.isfinite(current_pose).all() or np.asarray(current_pose).ndim != 1:
            raise ValueError('Starting configuration requires finite joint values')
        if not _in_ranges(current_pose, normalize_joint_ranges(options.get('joint_ranges'), len(current_pose))):
            raise ValueError('Starting configuration exceeds joint limits')
    ranked = []
    diagnostics = []
    for base in bases:
        check_cancel(options.get('cancel_check'))
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
    result.counts_complete = not fast
    if fast:
        result.ik_option_count = None
    tried = []
    previous_standoff = None
    previous_reason = None
    priority_targets = []
    checks = 0
    guess = None
    reported_attempt = None
    while ranked and len(tried) < attempts:
        check_cancel(options.get('cancel_check'))
        if checks % 100 == 0 or reported_attempt != len(tried):
            _stationary_progress(options, 'placement', tested=len(tried), remaining=len(ranked))
            reported_attempt = len(tried)
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
                    base_collision_detail=_collision_failure(base_collision),
                    ik_checked=False, targets_checked=0, reachable_targets=0, solution_counts=[],
                    path_checked=False, **metrics))
                continue
        if guess is None:
            guess = base
        tried.append(np.array(metrics['arm_origin'])[:2])
        previous_standoff = metrics['standoff']
        if current_pose is not None and collision is not None and not collision(current_pose, base):
            reason = _collision_failure(collision) or 'Starting configuration collision'
            result.initial_state_failure = reason
            diagnostics.append(dict(base_plane=base, reason='initial_collision',
                initial_state_failure=reason, ik_checked=False, targets_checked=0,
                reachable_targets=0, solution_counts=[], path_checked=False, **metrics))
            previous_reason = 'collision'
            continue
        candidate_options = dict(options, _priority_targets=priority_targets)
        if fast:
            candidate = _validate_stationary_fast(targets, base, current_pose,
                solver, collision, metrics, build_path, candidate_options)
        else:
            candidate = _find_stationary_base_by_options(targets, [base], current_pose,
                solver, collision, region, build_path, candidate_options)
        diagnostics.extend(candidate.diagnostics)
        result.initial_state_failure = None
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


def _collision_failure(collision):
    from functools import partial
    function = collision.func if isinstance(collision, partial) else collision
    return getattr(getattr(function, '__self__', None), 'last_failure', None)


def _stationary_progress(options, stage, **data):
    if options.get('progress') is not None:
        options['progress'](dict(stage=stage, **data))


def _stationary_disconnection(solved, layers, start, options):
    """Describe the first blocked transition using original target indices."""
    i = solved.failure_layer
    if solved.configurations or i is None:
        return None
    n = len(layers[i][0]) if layers[i] else (len(start) if start is not None else 0)
    limits = np.broadcast_to(options.get('max_joint_step', 2.5), (n,)).astype(float)
    if options.get('step_limits') is not None:
        limits = np.minimum(limits, options['step_limits'][i])
    previous = ([start] if i == 0 and start is not None else
                [layers[i-1][j] for j in solved.reachable_indices] if i else [])
    periodic = np.zeros(n, dtype=bool) if options.get('periodic') is None else np.asarray(options['periodic'], dtype=bool)
    best = None
    for q in previous:
        delta = np.asarray(layers[i])-q
        delta[:, periodic] = (delta[:, periodic]+np.pi) % (2*np.pi)-np.pi
        ratios = np.max(np.abs(delta)/np.maximum(limits, 1e-15), axis=1)
        j = int(np.argmin(ratios))
        if best is None or ratios[j] < best[0]:
            best = float(ratios[j]), np.abs(delta[j]).tolist()
    return dict(from_target=i-1, to_target=i, joint_step_limits=limits.tolist(),
        minimum_joint_step_limit_ratio=best[0] if best else None,
        joint_deltas_at_nearest_pair=best[1] if best else None,
        reason='No connected path under configuration and joint-step/speed constraints')


def _validate_stationary_fast(targets, base, start, solver, collision, geometry, build_path, options):
    """Prove per-target reachability before lazily validating the winner's path."""
    offsets = rotation_offsets(options.get('rotation_mode', False),
        options.get('rotation_angle_deg', 5), options.get('rotation_steps', 35),
        options.get('angle_cw_deg', 0), options.get('angle_ccw_deg', 0))
    timing = dict(ik_seconds=0., joint_expansion_seconds=0., collision_seconds=0.)
    diagnostic = dict(base_plane=base, cost=float('inf'), unreachable_points=[],
        solution_counts_before_filters=[], solution_counts=[], reachable_targets=0,
        path_count=0, ik_option_count=None, counts_complete=False, ik_checked=True,
        targets_checked=0, path_checked=False, reason='', timings=timing, **geometry)
    result = BasePlan([], [], float('inf'), [], [diagnostic], counts_complete=False)
    result.ik_option_count = None
    layers, details, checked, layer_angles = {}, {}, {}, {}

    def valid(i, j):
        check_cancel(options.get('cancel_check'))
        if (i, j) not in checked:
            tick = perf_counter()
            accepted = collision is None or collision(layers[i][j], base)
            timing['collision_seconds'] += perf_counter()-tick
            checked[i, j] = accepted
            stats = details[i]
            stats['collision_checks'] += int(collision is not None)
            if accepted:
                stats['collision_free'] += 1
            else:
                stats['collision_rejections'] += 1
                reason = _collision_failure(collision) or 'collision checker rejected configuration'
                failures = stats['rejection_reasons']
                failures[reason] = failures.get(reason, 0)+1
        return checked[i, j]

    order = list(dict.fromkeys(list(options.get('_priority_targets', [])) + list(range(len(targets)))))
    for i in order:
        check_cancel(options.get('cancel_check'))
        stats = {}
        angles = []
        rows, raw, _ = candidates(targets[i], base, solver, offsets, None,
            options.get('joint_ranges'), stats=stats, candidate_angles=angles,
            cancel_check=options.get('cancel_check'))
        layer_angles[i] = angles
        layers[i], details[i] = rows, stats
        timing['ik_seconds'] += stats['ik_seconds']
        timing['joint_expansion_seconds'] += stats['joint_expansion_seconds']
        stats.update(collision_free=0, collision_checks=0, collision_rejections=0,
            collision_check_applied=collision is not None,
            collision_validation='configurations_only' if collision is not None else 'not_checked')
        diagnostic['targets_checked'] += 1
        if not any(valid(i, j) for j in range(len(rows))):
            diagnostic['unreachable_points'] = [i]
            diagnostic['reason'] = ('no_ik' if not raw else
                'joint_limits' if not rows else 'collision')
            diagnostic['failed_target_details'] = dict(target_index=i, **stats)
            break
        diagnostic['reachable_targets'] += 1
        if diagnostic['targets_checked'] % 100 == 0 or diagnostic['targets_checked'] == len(targets):
            _stationary_progress(options, 'ik_candidates', tested=diagnostic['targets_checked'], total=len(targets))
    diagnostic['checked_target_indices'] = sorted(layers)
    diagnostic['solution_counts_before_filters'] = [details[i]['raw_ik'] for i in sorted(layers)]
    if not diagnostic['unreachable_points']:
        result.base_planes = [base]
        result.standoff = geometry.get('standoff', 0.)
        result.max_target_distance = geometry.get('max_target_distance', 0.)
        diagnostic['reason'] = 'all_targets_reachable'
        if build_path:
            tick = perf_counter()
            graph_stats = {}
            # Reachability probes already proved some nodes invalid. Remove
            # those before the first graph solve, retaining candidate order.
            indices = [[j for j in range(len(layers[i])) if checked.get((i, j), True)]
                       for i in range(len(targets))]
            def graph_valid(i, j):
                return valid(i, indices[i][j])
            def rejected_layer(i, j):
                rejected = [k for k in range(len(indices[i])) if not graph_valid(i, k)]
                _stationary_progress(options, 'collision_candidates', target=i, rejected=len(rejected))
                return rejected
            graph_layers = [[layers[i][j] for j in ids] for i, ids in enumerate(indices)]
            _stationary_progress(options, 'joint_graph', total=len(targets))
            solved = shortest_path(graph_layers, start=start,
                max_step=options.get('max_joint_step', 2.5), periodic=options.get('periodic'),
                weights=options.get('weights'), count_paths=False,
                revolute_joints=getattr(solver, 'revolute_joints', None),
                node_valid=graph_valid, node_rejection_group=rejected_layer, stats=graph_stats,
                step_limits=options.get('step_limits'),
                cancel_check=lambda: check_cancel(options.get('cancel_check')))
            result.disconnected_detail = _stationary_disconnection(solved, graph_layers, start, options)
            timing['path_seconds'] = graph_stats.get('graph_seconds', perf_counter()-tick)
            diagnostic['graph_solves'] = graph_stats.get('graph_solves', 0)
            result.configurations, result.cost = solved.configurations, solved.cost
            if solved.configurations:
                result.selected_tcp_rotations = [layer_angles[i][indices[i][j]]
                                                 for i, j in enumerate(solved.indices)]
                result.selected_target_planes = [as_plane(t).rotated_z(a)
                    for t, a in zip(targets, result.selected_tcp_rotations)]
            result.path_search_count = 1
            diagnostic.update(path_checked=True, cost=solved.cost,
                reason='complete' if solved.configurations else 'joint_step_disconnected')
    diagnostic['collision_checks'] = sum(d['collision_checks'] for d in details.values())
    return result


def _find_stationary_base_by_options(targets, bases, current_pose, solver, collision,
                                     region, build_path, options):
    """Configuration-only ranking with early rejection and one winner's graph."""
    offsets = rotation_offsets(options.get('rotation_mode', False),
        options.get('rotation_angle_deg', 5), options.get('rotation_steps', 35),
        options.get('angle_cw_deg', 0), options.get('angle_ccw_deg', 0))
    best = BasePlan([], [], float('inf'), [], [])
    best_layers, best_rank, selected, best_angles = None, None, None, None
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
        layers, raw, layer_angles = {}, {}, {}
        diagnostic['timings'] = dict(ik_seconds=0.0, joint_expansion_seconds=0.0, collision_seconds=0.0)
        priority = options.get('_priority_targets', [])
        order = list(dict.fromkeys(list(priority) + list(range(len(targets)))))
        for i in order:
            check_cancel(options.get('cancel_check'))
            target = targets[i]
            stats = {}
            angles = []
            qs, before, _ = candidates(target, base, solver, offsets, collision,
                options.get('joint_ranges'), stats=stats, candidate_angles=angles,
                cancel_check=options.get('cancel_check'))
            layer_angles[i] = angles
            for name in diagnostic['timings']:
                diagnostic['timings'][name] += stats[name]
            layers[i] = qs
            raw[i] = before
            if len(layers) % 100 == 0 or len(layers) == len(targets):
                _stationary_progress(options, 'ik_candidates', tested=len(layers), total=len(targets))
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
            best_angles = [layer_angles[i] for i in checked]
            best.base_planes, best.candidate_counts = [base], counts
            best.ik_option_count, best.standoff = combinations, standoff
            best.max_target_distance = geometry.get('max_target_distance', 0.0)
    if best_layers is not None and build_path:
        _stationary_progress(options, 'joint_graph', total=len(targets))
        path_started = perf_counter()
        solved = shortest_path(best_layers, start=current_pose,
            max_step=options.get('max_joint_step', 2.5), periodic=options.get('periodic'),
            weights=options.get('weights'), count_paths=options.get('count_paths', True),
            revolute_joints=getattr(solver, 'revolute_joints', None),
            step_limits=options.get('step_limits'),
            cancel_check=lambda: check_cancel(options.get('cancel_check')))
        best.disconnected_detail = _stationary_disconnection(solved, best_layers, current_pose, options)
        selected['timings']['path_seconds'] = perf_counter()-path_started
        best.configurations, best.cost, best.path_count = solved.configurations, solved.cost, solved.path_count
        if solved.configurations:
            best.selected_tcp_rotations = [best_angles[i][j] for i, j in enumerate(solved.indices)]
            best.selected_target_planes = [as_plane(t).rotated_z(a)
                for t, a in zip(targets, best.selected_tcp_rotations)]
        best.path_search_count = 1
        selected.update(path_checked=True, path_count=solved.path_count, cost=solved.cost,
            reason='complete' if solved.configurations else 'joint_step_disconnected')
    return best

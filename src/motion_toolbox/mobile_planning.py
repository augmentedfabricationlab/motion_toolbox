"""Optional sparse mobile proposals with full-resolution feasibility checks."""
import math
from time import perf_counter
import numpy as np
from .geometry import Plane, as_plane
from .base_planning import BasePlan, plan_mobile_base


def mobile_base_seeds(targets, *, distance=1.0, height=0.0):
    """Upright footprint proposals behind projected TCP +Z, in metres.

    For vertical TCP normals use projected TCP +X instead. These geometric
    seeds are not reachability/collision guarantees; the robot planner validates
    the candidate path using the actual arm mounting and tool.
    """
    if not math.isfinite(distance) or distance <= 0 or not math.isfinite(height):
        raise ValueError('Seed distance must be positive and height finite')
    seeds = []
    for target in targets:
        target = as_plane(target)
        direction = target.zaxis[:2]
        if np.linalg.norm(direction) < 1e-8:
            direction = target.xaxis[:2]
        direction = direction / np.linalg.norm(direction)
        x, y = target.origin[:2] - distance*direction
        dx, dy = direction
        seeds.append(Plane((x,y,height), (dx,dy,0), (-dy,dx,0)))
    return seeds


def plan_mobile_robot_path(targets, seeds, settings, *, rotation_steps=1, base_collision=None, **options):
    """Robot adapter: optimize around existing geometric footprint path seeds.

    settings: sparse (False), xy_offsets ([[0,0]]) in world XY metres,
    yaw_offsets ([0]) radians, max_gap/max_distance/angle for sparse selection,
    and plan_mobile_base limits/weights/timing/start_base (numeric metre plane).
    Supplied seeds contain one plane or one per target. current_pose requires
    an explicit start_base, so the approach is never silently assumed.
    """
    from .planning import rotation_offsets
    settings = dict(settings)
    allowed = {'strategy', 'lateral_distance', 'wall_distances', 'smoothing_windows',
               'lateral_offsets', 'smooth_max_attempts',
               'sparse', 'xy_offsets', 'yaw_offsets', 'max_gap', 'max_distance', 'angle',
               'sampling', 'xy_tolerance', 'normal_angle', 'z_tolerance',
               'start_base', 'max_base_step', 'max_yaw_step', 'base_weight', 'yaw_weight',
               'joint_weights', 'time_intervals', 'max_base_speed', 'max_yaw_speed', 'max_joint_speed',
               'placement_region', 'grid_spacing', 'yaw_steps', 'base_height', 'max_feasible_bases'}
    unknown = set(settings)-allowed
    if unknown:
        raise ValueError('Unknown mobile options: ' + ', '.join(sorted(unknown)))
    strategy = settings.pop('strategy', 'discrete')
    if strategy not in ('discrete', 'smooth_offset'):
        raise ValueError('strategy must be discrete or smooth_offset')
    smooth = {name: settings.pop(key) for key,name in (
        ('lateral_distance','lateral_distance'), ('wall_distances','wall_distances'),
        ('smoothing_windows','windows'), ('lateral_offsets','lateral_offsets'),
        ('smooth_max_attempts','max_attempts')) if key in settings}
    if smooth and strategy != 'smooth_offset':
        raise ValueError('Smooth path settings require strategy=smooth_offset')
    sparse = settings.pop('sparse', False)
    use_region = settings.pop('placement_region', False)
    spacing = settings.pop('grid_spacing', .5)
    yaw_steps = settings.pop('yaw_steps', 4)
    height = settings.pop('base_height', 0.)
    xy = np.asarray(settings.pop('xy_offsets', [[0, 0]]), dtype=float)
    yaw = np.asarray(settings.pop('yaw_offsets', [0]), dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not len(xy) or not np.isfinite(xy).all():
        raise ValueError('xy_offsets requires finite world XY pairs in metres')
    if yaw.ndim != 1 or not len(yaw) or not np.isfinite(yaw).all():
        raise ValueError('yaw_offsets requires finite angles in radians')
    selection = {k: settings.pop(k) for k in ('max_gap', 'max_distance', 'angle',
                 'sampling', 'xy_tolerance', 'normal_angle', 'z_tolerance') if k in settings}
    if selection and not sparse:
        raise ValueError('Keyframe settings require sparse=true')
    options.update(settings)
    options['offsets'] = rotation_offsets('n_steps', steps=rotation_steps)
    regions, body_cache = {}, {}
    def region(target):
        from .stationary_region import StationaryRegion
        # Input planes remain alive/immutable throughout this invocation.
        # Avoid reconstructing a 4x4 matrix on every base-candidate lookup.
        key = id(target)
        if key not in regions:
            regions[key] = StationaryRegion([target], options['ik_solver'].arm_in_base,
                max_distance=1.75, base_height=height, projected=True)
        return regions[key]
    if use_region:
        def valid(target, base):
            if not region(target).metrics(base)['geometry_valid']:
                return False
            key = base.matrix.tobytes()
            if key not in body_cache:
                body_cache[key] = base_collision is None or base_collision(base)
            return body_cache[key]
        options['base_valid'] = valid
        if sparse:
            options.setdefault('max_feasible_bases', 4)
    def layer(i, target):
        if use_region:
            generated, _, _ = region(target).candidates(spacing=spacing, yaw_steps=yaw_steps)
            # Supplied seeds are optional proposals, subject to the same rules.
            return ([seeds[0] if len(seeds) == 1 else seeds[i]] if seeds else []) + generated
        seed = seeds[0] if len(seeds) == 1 else seeds[i]
        return [Plane(seed.origin + np.r_[offset, 0.], seed.xaxis, seed.yaxis).rotated_z(a)
                for offset in xy for a in yaw]
    started = perf_counter()
    if strategy == 'smooth_offset':
        from .smooth_mobile import plan_smooth_mobile
        if not use_region:
            raise ValueError('smooth_offset requires placement_region=true')
        solved = plan_smooth_mobile(targets, options['ik_solver'].arm_in_base,
                                   height=height, **smooth, **options)
    else:
        solved = (plan_mobile_sparse(targets, layer, **selection, **options) if sparse else
                  plan_mobile_base(targets, [layer(i,t) for i,t in enumerate(targets)], **options))
    return dict(configurations=solved.configurations, base_planes=solved.base_planes,
        path_length=solved.cost, num_nodes_computed=len(targets),
        unreachable_points=[i for i,n in enumerate(solved.candidate_counts) if n == 0],
        unchecked_points=[i for i,n in enumerate(solved.candidate_counts) if n is None],
        collision_check_applied=options.get('collision') is not None,
        target_diagnostics=solved.target_diagnostics, mobile_diagnostics=solved.diagnostics,
        ik_solutions_per_node=solved.ik_solutions_per_node,
        solution_counts_after_collision=solved.candidate_counts,
        timings={'mobile_seconds': perf_counter()-started})


def significant_targets(targets, *, max_gap=20, max_distance=0.2, angle=0.15):
    """Keep endpoints, turns, accumulated orientation changes and bounded gaps.

    Lengths are metres and angles radians. This linear-time selector is a
    heuristic, not a reachability test or a geometric error bound.
    """
    if int(max_gap) != max_gap or max_gap < 1:
        raise ValueError('max_gap must be a positive integer')
    if any(not math.isfinite(v) or v <= 0 for v in (max_distance, angle)):
        raise ValueError('max_distance and angle must be positive and finite')
    frames = [as_plane(t) for t in targets]
    if not frames:
        raise ValueError('At least one target required')
    kept, distance, turn = [0], 0.0, 0.0
    previous_direction = None
    for i in range(1, len(frames)):
        delta = frames[i].origin - frames[i-1].origin
        length = float(np.linalg.norm(delta))
        if length > 1e-12:
            direction = delta / length
            if previous_direction is not None:
                turn += math.acos(float(np.clip(direction @ previous_direction, -1, 1)))
                if turn >= angle and i-1 > kept[-1]:
                    kept.append(i-1)
                    distance, turn = 0.0, 0.0
            previous_direction = direction
        distance += length
        relative = frames[kept[-1]].matrix[:3, :3].T @ frames[i].matrix[:3, :3]
        rotation = math.acos(float(np.clip((np.trace(relative)-1)/2, -1, 1)))
        if i-kept[-1] >= max_gap or distance >= max_distance or rotation >= angle:
            kept.append(i)
            distance, turn = 0.0, 0.0
    if kept[-1] != len(frames)-1:
        kept.append(len(frames)-1)
    return kept


def xy_feature_targets(targets, *, xy_tolerance=.05, normal_angle=.35,
                       z_tolerance=None, max_gap=None, max_distance=None):
    """Simplify ordered world-XY positions by bounded point-to-segment error.

    Iterative Douglas-Peucker retains large corners, reversals and excursions,
    including paths whose endpoints coincide. Small ripples within tolerance
    do not force points merely because they turn sharply. Z and TCP roll/pitch
    are ignored by default. Projected wall-normal heading changes are retained
    at normal_angle radians (None disables); z_tolerance optionally bounds
    height changes from an anchor. Lengths are metres. Optional gap/distance
    caps add points after simplification; they are disabled by default.
    """
    for name, value in (('xy_tolerance', xy_tolerance), ('normal_angle', normal_angle),
                        ('z_tolerance', z_tolerance), ('max_distance', max_distance)):
        if value is None and name != 'xy_tolerance':
            continue
        if value is None or not math.isfinite(value) or value <= 0:
            raise ValueError(name+' must be positive and finite')
    if max_gap is not None and (int(max_gap) != max_gap or max_gap < 1):
        raise ValueError('max_gap must be a positive integer or None')
    frames = [as_plane(t) for t in targets]
    if not frames:
        raise ValueError('At least one target required')
    points = np.array([t.origin[:2] for t in frames])
    keep = {0, len(frames)-1}
    anchor = 0
    for i in range(1, len(frames)):
        changed = z_tolerance is not None and abs(frames[i].origin[2]-frames[anchor].origin[2]) >= z_tolerance
        if normal_angle is not None:
            a, b = frames[anchor].zaxis[:2], frames[i].zaxis[:2]
            if min(np.linalg.norm(a), np.linalg.norm(b)) > 1e-9:
                changed |= math.acos(float(np.clip(a @ b / (np.linalg.norm(a)*np.linalg.norm(b)), -1, 1))) >= normal_angle
        if changed:
            keep.add(i)
            anchor = i
    # Optional safeguards use XY displacement, never accumulated ripple length.
    if max_gap is not None or max_distance is not None:
        anchor = 0
        for i in range(1, len(frames)):
            if (i in keep or (max_gap is not None and i-anchor >= max_gap) or
                    (max_distance is not None and np.linalg.norm(points[i]-points[anchor]) >= max_distance)):
                keep.add(i)
                anchor = i
    ordered = sorted(keep)
    stack = list(zip(ordered, ordered[1:]))
    while stack:
        a, b = stack.pop()
        if b-a < 2:
            continue
        chord = points[b]-points[a]
        length2 = float(chord @ chord)
        delta = points[a+1:b]-points[a]
        fraction = np.clip(delta @ chord/length2, 0, 1) if length2 > 1e-24 else np.zeros(len(delta))
        error2 = np.sum((delta-fraction[:,None]*chord)**2, axis=1)
        offset = int(np.argmax(error2))
        if error2[offset] > xy_tolerance**2:
            i = a+1+offset
            keep.add(i)
            stack.extend(((a,i),(i,b)))
    return sorted(keep)


def xy_feature_progress(targets, indices):
    """Monotone progress along simplified XY segments, ignoring ripple length."""
    points = np.array([as_plane(t).origin[:2] for t in targets])
    coordinates = np.zeros(len(points))
    for a,b in zip(indices, indices[1:]):
        chord = points[b]-points[a]
        length = float(np.linalg.norm(chord))
        fraction = (np.maximum.accumulate(np.clip((points[a:b+1]-points[a]) @ chord/length**2,0,1))
                    if length > 1e-12 else np.linspace(0,1,b-a+1))
        coordinates[a:b+1] = coordinates[a] + fraction*(length if length > 1e-12 else 1.)
    return coordinates


def interpolate_bases(indices, bases, coordinates):
    """Linear translation and shortest-arc upright yaw, at every coordinate."""
    result = [None]*len(coordinates)
    for index, base in zip(indices, bases):
        result[index] = base
    for a, b, first, last in zip(indices, indices[1:], bases, bases[1:]):
        yaw = math.atan2(first.xaxis[1], first.xaxis[0])
        dyaw = (math.atan2(last.xaxis[1], last.xaxis[0])-yaw+math.pi) % (2*math.pi)-math.pi
        for i in range(a+1, b):
            span = coordinates[b]-coordinates[a]
            fraction = (coordinates[i]-coordinates[a])/span if span > 0 else (i-a)/(b-a)
            heading = yaw + fraction*dyaw
            result[i] = Plane(first.origin + fraction*(last.origin-first.origin),
                              (math.cos(heading), math.sin(heading), 0),
                              (-math.sin(heading), math.cos(heading), 0))
    return result


def plan_mobile_sparse(targets, base_candidates_per_target, *, max_gap=None,
                       max_distance=None, angle=0.15, sampling='xy', xy_tolerance=.05,
                       normal_angle=.35, z_tolerance=None, **options):
    """Search keyframes, validate every original pose/edge; dense fallback.

    Candidate layers may be a callable(index, target), evaluated lazily. Sparse
    interpolation may introduce placements outside the discrete candidate set;
    use collision/transition callbacks for hard workspace/steering constraints.
    This mode is approximate. Use plan_mobile_base for the discrete optimum.
    All original limits and collision callbacks apply to full-resolution output.
    """
    started = perf_counter()
    targets = [as_plane(t) for t in targets]
    # Same target/base candidate states are independent of coarse edge limits.
    # Keep this cache local to this invocation and its fixed robot/scene/options.
    options = dict(options, _candidate_cache={})
    if sampling == 'xy':
        indices = xy_feature_targets(targets, xy_tolerance=xy_tolerance,
            normal_angle=normal_angle, z_tolerance=z_tolerance, max_gap=max_gap, max_distance=max_distance)
    elif sampling == 'legacy':
        indices = significant_targets(targets, max_gap=20 if max_gap is None else max_gap,
            max_distance=.2 if max_distance is None else max_distance, angle=angle)
    else:
        raise ValueError("sampling must be 'xy' or 'legacy'")
    selection_info = dict(sampling=sampling, keyframe_count=len(indices), target_count=len(targets),
                          xy_tolerance=xy_tolerance if sampling == 'xy' else None)
    if not callable(base_candidates_per_target) and len(base_candidates_per_target) != len(targets):
        raise ValueError('Provide one candidate layer per target')
    cache = {}
    def layer(i):
        if i not in cache:
            cache[i] = list(base_candidates_per_target(i, targets[i]) if callable(base_candidates_per_target)
                            else base_candidates_per_target[i])
        return cache[i]
    times = options.get('time_intervals')
    has_start = options.get('start_base') is not None
    coordinates = np.arange(len(targets), dtype=float)
    if times is not None:
        expected = len(targets) if has_start else len(targets)-1
        if len(times) != expected or any(not math.isfinite(t) or t <= 0 for t in times):
            raise ValueError('One positive duration per edge required')
        coordinates = np.r_[0., np.cumsum(times[1:] if has_start else times)]
    elif any(options.get(k) is not None for k in ('max_base_speed', 'max_yaw_speed', 'max_joint_speed')):
        raise ValueError('Speed constraints require time_intervals')
    else:
        coordinates = (xy_feature_progress(targets, indices) if sampling == 'xy' else
            np.r_[0., np.cumsum([np.linalg.norm(b.origin-a.origin) for a,b in zip(targets, targets[1:])])])
    fallback = False
    if len(indices) < len(targets):
        # Endpoint joint changes need not satisfy a single original-step bound.
        # Intermediate collisions must be checked along the actual dense path.
        coarse_options = dict(options, max_base_step=None, max_yaw_step=None,
                              max_joint_step=None, transition_check=None, _stop_on_unreachable=True)
        if times is not None:
            coarse_options['time_intervals'] = ([times[0]] if has_start else []) + [
                float(coordinates[b]-coordinates[a]) for a,b in zip(indices, indices[1:])]
        # Lazy layers avoid generating later regions after an impossible keyframe.
        class KeyframeLayers:
            def __len__(self):
                return len(indices)
            def __iter__(self):
                return (layer(i) for i in indices)
        proposal = plan_mobile_base([targets[i] for i in indices], KeyframeLayers(), **coarse_options)
        if any(n == 0 for n in proposal.candidate_counts):
            counts = [None]*len(targets)
            details = [dict(checked=False) for _ in targets]
            joints = [[] for _ in targets]
            for j,i in enumerate(indices):
                counts[i] = proposal.candidate_counts[j]
                details[i] = proposal.target_diagnostics[j]
                joints[i] = proposal.ik_solutions_per_node[j]
            return BasePlan([], [], float('inf'), counts,
                [dict(mode='sparse', keyframe_indices=indices, dense_fallback=False,
                      reason='keyframe_has_no_feasible_candidate',
                      checked_target_count=sum(n is not None for n in counts),
                      total_seconds=perf_counter()-started, **selection_info)],
                ik_solutions_per_node=joints, target_diagnostics=details)
        if proposal.base_planes:
            bases = interpolate_bases(indices, proposal.base_planes, coordinates)
            result = plan_mobile_base(targets, [[b] for b in bases], **options)
            if result.configurations:
                result.diagnostics.append(dict(mode='sparse', keyframe_indices=indices,
                    dense_fallback=False, total_seconds=perf_counter()-started, **selection_info))
                return result
        fallback = True
    class DenseLayers:
        def __len__(self):
            return len(targets)
        def __iter__(self):
            return (layer(i) for i in range(len(targets)))
    # Only bounded candidate search needs continuity-aware pruning. Explicit
    # uncapped search retains its exact supplied-domain semantics.
    connected = options.get('max_feasible_bases') is not None
    limits, repairs = {}, []
    for attempt in range(3):
        result = plan_mobile_base(targets, DenseLayers(), **dict(options,
            _connected_candidates=connected, _base_limits=limits, _stop_on_unreachable=True))
        blocked = next((d for d in result.diagnostics if d.get('reason') == 'transition_blocked'), None)
        if result.configurations or not connected or blocked is None or attempt == 2:
            break
        boundary = blocked['to_target']
        cap = max(options['max_feasible_bases'], 16 if attempt == 0 else 64)
        for j in range(max(0, boundary-2), boundary+1):
            limits[j] = cap
        repairs.append(dict(blocked_transition=blocked, expanded_targets=sorted(limits), base_limit=cap))
    if repairs:
        result.diagnostics.append(dict(local_repairs=repairs))
    result.diagnostics.append(dict(mode='dense', keyframe_indices=indices,
        dense_fallback=fallback, total_seconds=perf_counter()-started, **selection_info))
    return result

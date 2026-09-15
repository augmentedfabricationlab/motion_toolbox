"""Optional sparse mobile proposals with full-resolution feasibility checks."""
import math
from time import perf_counter
import numpy as np
from .geometry import Plane, as_plane
from .base_planning import plan_mobile_base


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


def plan_mobile_robot_path(targets, seeds, settings, *, rotation_steps=1, **options):
    """Robot adapter: optimize around existing geometric footprint path seeds.

    settings: sparse (False), xy_offsets ([[0,0]]) in world XY metres,
    yaw_offsets ([0]) radians, max_gap/max_distance/angle for sparse selection,
    and plan_mobile_base limits/weights/timing/start_base (numeric metre plane).
    Supplied seeds contain one plane or one per target. current_pose requires
    an explicit start_base, so the approach is never silently assumed.
    """
    from .planning import rotation_offsets
    settings = dict(settings)
    allowed = {'sparse', 'xy_offsets', 'yaw_offsets', 'max_gap', 'max_distance', 'angle',
               'start_base', 'max_base_step', 'max_yaw_step', 'base_weight', 'yaw_weight',
               'joint_weights', 'time_intervals', 'max_base_speed', 'max_yaw_speed', 'max_joint_speed'}
    unknown = set(settings)-allowed
    if unknown:
        raise ValueError('Unknown mobile options: ' + ', '.join(sorted(unknown)))
    sparse = settings.pop('sparse', False)
    xy = np.asarray(settings.pop('xy_offsets', [[0, 0]]), dtype=float)
    yaw = np.asarray(settings.pop('yaw_offsets', [0]), dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not len(xy) or not np.isfinite(xy).all():
        raise ValueError('xy_offsets requires finite world XY pairs in metres')
    if yaw.ndim != 1 or not len(yaw) or not np.isfinite(yaw).all():
        raise ValueError('yaw_offsets requires finite angles in radians')
    selection = {k: settings.pop(k) for k in ('max_gap', 'max_distance', 'angle') if k in settings}
    if selection and not sparse:
        raise ValueError('Keyframe settings require sparse=true')
    options.update(settings)
    options['offsets'] = rotation_offsets('n_steps', steps=rotation_steps)
    def layer(i, target):
        seed = seeds[0] if len(seeds) == 1 else seeds[i]
        return [Plane(seed.origin + np.r_[offset, 0.], seed.xaxis, seed.yaxis).rotated_z(a)
                for offset in xy for a in yaw]
    started = perf_counter()
    solved = (plan_mobile_sparse(targets, layer, **selection, **options) if sparse else
              plan_mobile_base(targets, [layer(i,t) for i,t in enumerate(targets)], **options))
    return dict(configurations=solved.configurations, base_planes=solved.base_planes,
        path_length=solved.cost, num_nodes_computed=len(targets),
        unreachable_points=[i for i,n in enumerate(solved.candidate_counts) if not n],
        collision_check_applied=options.get('collision') is not None,
        target_diagnostics=[], mobile_diagnostics=solved.diagnostics,
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


def plan_mobile_sparse(targets, base_candidates_per_target, *, max_gap=20,
                       max_distance=0.2, angle=0.15, **options):
    """Search keyframes, validate every original pose/edge; dense fallback.

    Candidate layers may be a callable(index, target), evaluated lazily. Sparse
    interpolation may introduce placements outside the discrete candidate set;
    use collision/transition callbacks for hard workspace/steering constraints.
    This mode is approximate. Use plan_mobile_base for the discrete optimum.
    All original limits and collision callbacks apply to full-resolution output.
    """
    started = perf_counter()
    targets = [as_plane(t) for t in targets]
    indices = significant_targets(targets, max_gap=max_gap, max_distance=max_distance, angle=angle)
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
        coordinates = np.r_[0., np.cumsum([np.linalg.norm(b.origin-a.origin) for a,b in zip(targets, targets[1:])])]
    fallback = False
    if len(indices) < len(targets):
        # Endpoint joint changes need not satisfy a single original-step bound.
        # Intermediate collisions must be checked along the actual dense path.
        coarse_options = dict(options, max_base_step=None, max_yaw_step=None,
                              max_joint_step=None, transition_check=None)
        if times is not None:
            coarse_options['time_intervals'] = ([times[0]] if has_start else []) + [
                float(coordinates[b]-coordinates[a]) for a,b in zip(indices, indices[1:])]
        proposal = plan_mobile_base([targets[i] for i in indices], [layer(i) for i in indices], **coarse_options)
        if proposal.base_planes:
            bases = interpolate_bases(indices, proposal.base_planes, coordinates)
            result = plan_mobile_base(targets, [[b] for b in bases], **options)
            if result.configurations:
                result.diagnostics.append(dict(mode='sparse', keyframe_indices=indices,
                    dense_fallback=False, total_seconds=perf_counter()-started))
                return result
        fallback = True
    result = plan_mobile_base(targets, [layer(i) for i in range(len(targets))], **options)
    result.diagnostics.append(dict(mode='dense', keyframe_indices=indices,
        dense_fallback=fallback, total_seconds=perf_counter()-started))
    return result

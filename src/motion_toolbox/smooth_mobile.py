"""Smooth wall-offset proposals followed by full-resolution robot validation."""
from time import perf_counter
import numpy as np
from .geometry import Plane, as_plane
from .base_planning import BasePlan, plan_mobile_base


def moving_average(values, window):
    """Centred, endpoint-padded average in O(n), including even windows."""
    left = (window-1)//2
    padded = np.pad(values, ((left, window-1-left), (0, 0)), mode='edge')
    sums = np.vstack((np.zeros((1, values.shape[1])), np.cumsum(padded, axis=0)))
    return (sums[window:]-sums[:-window])/window


def smooth_offset_proposals(targets, arm_in_base, *, height=0., lateral_distance=1.,
                            windows=(10, 25, 50, 100, 200),
                            wall_distances=(.4, .6, .8, 1., 1.2), lateral_offsets=None,
                            _defer_planes=False):
    """Rank smooth whole paths; no per-target greedy footprint decisions.

    TCP projected +Z points from robot to wall. Base +X faces that direction.
    Sideways offsets are along footprint +/-Y, wall distance along +X.
    Reach checks include the calibrated arm mounting translation. Local normal
    offsets follow curved walls without assuming a circular workpiece.
    """
    targets = [as_plane(t) for t in targets]
    if not targets:
        raise ValueError('At least one target required')
    windows = list(windows)
    if not windows or any(int(w) != w or w < 1 for w in windows):
        raise ValueError('smoothing_windows must contain positive integers')
    if not np.isfinite(lateral_distance) or lateral_distance < 0:
        raise ValueError('lateral_distance must be finite and nonnegative')
    distances = list(wall_distances)
    if not distances or any(not np.isfinite(d) or d <= 0 for d in distances):
        raise ValueError('wall_distances must contain positive finite metres')
    lateral_offsets = list(lateral_offsets if lateral_offsets is not None else (-lateral_distance, lateral_distance))
    if not len(lateral_offsets) or not np.isfinite(lateral_offsets).all():
        raise ValueError('lateral_offsets must contain finite metres')
    xy = np.array([t.origin[:2] for t in targets])
    normals = np.array([t.zaxis[:2] for t in targets])
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths < 1e-8):
        raise ValueError('Smooth wall-offset planning requires target normals with a world-XY component')
    normals /= lengths[:, None]
    mount = as_plane(arm_in_base).origin
    proposals, rejected = [], []
    for window in sorted(set(min(int(w), len(targets)) for w in windows)):
        centre = moving_average(xy, window)
        direction = moving_average(normals, window)
        norms = np.linalg.norm(direction, axis=1)
        if np.any(norms < 1e-6):
            rejected.append(dict(window=window, reason='opposing_wall_normals'))
            continue
        direction /= norms[:, None]
        tangent = np.column_stack((-direction[:, 1], direction[:, 0]))
        for offset in distances:
            for lateral in lateral_offsets:
                footprint = centre-offset*direction+lateral*tangent
                arm_xy = footprint+mount[0]*direction+mount[1]*tangent
                relative = xy-arm_xy
                reach = np.linalg.norm(relative, axis=1)
                side = np.einsum('ij,ij->i', relative, normals)
                bad = np.flatnonzero((reach > 1.75+1e-10) | (side <= 0))
                meta = dict(window=window, wall_distance_metres=float(offset), lateral_metres=float(lateral))
                if len(bad):
                    rejected.append(dict(meta, reason='placement_geometry', first_failed_target=int(bad[0])))
                    continue
                yaw = np.unwrap(np.arctan2(direction[:, 1], direction[:, 0]))
                curvature = float(np.mean(np.linalg.norm(np.diff(footprint, n=2, axis=0), axis=1))) if len(xy) > 2 else 0.
                yaw_roughness = float(np.mean(abs(np.diff(yaw, n=2)))) if len(xy) > 2 else 0.
                deviation = float(np.mean(np.linalg.norm(centre-xy, axis=1)))
                # Penalize acceleration/jitter, not total travel. Keep broad
                # features by also penalizing departure from the TCP trend.
                score = curvature+yaw_roughness+.01*deviation+.001*abs(abs(lateral)-lateral_distance)
                bases = ((footprint, direction, tangent) if _defer_planes else
                         [Plane((p[0], p[1], height), (*d, 0), (*t, 0))
                          for p,d,t in zip(footprint, direction, tangent)])
                proposals.append((score, bases, dict(meta, smoothness_score=score,
                    xy_second_difference=curvature, yaw_second_difference=yaw_roughness,
                    max_arm_xy_reach=float(reach.max()))))
    proposals.sort(key=lambda item: item[0])
    return proposals, rejected


def plan_smooth_mobile(targets, arm_in_base, *, height=0., lateral_distance=1., windows=(10,25,50,100,200),
                       wall_distances=(.4,.6,.8,1.,1.2), lateral_offsets=None, max_attempts=12, **options):
    """Try a bounded set of globally smooth paths with every original TCP checked."""
    if int(max_attempts) != max_attempts or max_attempts < 1:
        raise ValueError('smooth_max_attempts must be a positive integer')
    started = perf_counter()
    proposals, rejected = smooth_offset_proposals(targets, arm_in_base, height=height,
        lateral_distance=lateral_distance, windows=windows, wall_distances=wall_distances,
        lateral_offsets=lateral_offsets, _defer_planes=True)
    history = []
    result = BasePlan([], [], float('inf'), [None]*len(targets), [],
        target_diagnostics=[dict(checked=False) for _ in targets],
        ik_solutions_per_node=[[] for _ in targets])
    options = dict(options, _candidate_cache={}, _stop_on_unreachable=True)
    for _, arrays, metadata in proposals[:int(max_attempts)]:
        bases = [Plane((p[0], p[1], height), (*d, 0), (*t, 0)) for p,d,t in zip(*arrays)]
        result = plan_mobile_base(targets, [[b] for b in bases], **options)
        failed = next((i for i,n in enumerate(result.candidate_counts) if n == 0), None)
        history.append(dict(metadata, complete=bool(result.configurations),
            first_infeasible_target=failed,
            failed_target_details=result.target_diagnostics[failed] if failed is not None else None,
            transition_diagnostics=list(result.diagnostics)))
        if result.configurations:
            break
    result.diagnostics.append(dict(mode='smooth_offset', attempts=history,
        geometry_rejected_count=len(rejected), geometry_rejection_examples=rejected[:5],
        proposal_count=len(proposals), attempt_limit=int(max_attempts),
        selected=history[-1] if result.configurations else None,
        reason='complete' if result.configurations else 'smooth_proposals_exhausted',
        scope='bounded smooth proposals; failure does not prove global infeasibility',
        total_seconds=perf_counter()-started))
    return result

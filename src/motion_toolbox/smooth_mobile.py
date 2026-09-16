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
                       wall_distances=(.4,.6,.8,1.,1.2), lateral_offsets=None, max_attempts=12, repair_attempts=8, connect_sections=True,
                       section_size=100, section_proposals=6, section_beam_width=2, **options):
    """Try a bounded set of globally smooth paths with every original TCP checked."""
    if int(max_attempts) != max_attempts or max_attempts < 1:
        raise ValueError('smooth_max_attempts must be a positive integer')
    if int(repair_attempts) != repair_attempts or repair_attempts < 0:
        raise ValueError("smooth_repair_attempts must be a nonnegative integer")
    started = perf_counter()
    proposals, rejected = smooth_offset_proposals(targets, arm_in_base, height=height,
        lateral_distance=lateral_distance, windows=windows, wall_distances=wall_distances,
        lateral_offsets=lateral_offsets, _defer_planes=True)
    history = []
    result = BasePlan([], [], float('inf'), [None]*len(targets), [],
        target_diagnostics=[dict(checked=False) for _ in targets],
        ik_solutions_per_node=[[] for _ in targets])
    options = dict(options, _candidate_cache={}, _stop_on_unreachable=True)
    # Round-robin across smoothing windows and sides before spending the
    # remaining budget on more distances from the same family.
    groups = {}
    for proposal in proposals:
        meta = proposal[2]
        groups.setdefault((meta['window'], np.sign(meta['lateral_metres'])), []).append(proposal)
    # Start each family near one metre of wall clearance, then explore other
    # clearances. Tiny score differences must not exclude useful distances.
    for group in groups.values():
        group.sort(key=lambda p: (abs(p[2]['wall_distance_metres']-1.), p[0]))
    ordered = []
    while any(groups.values()):
        for group in groups.values():
            if group:
                ordered.append(group.pop(0))
    best = None
    checked_any = set()

    def evaluate(arrays, metadata):
        bases = [Plane((p[0], p[1], height), (*d, 0), (*t, 0)) for p,d,t in zip(*arrays)]
        solved = plan_mobile_base(targets, [[b] for b in bases], **options)
        checked_any.update(i for i,n in enumerate(solved.candidate_counts) if n is not None)
        failed = next((i for i,n in enumerate(solved.candidate_counts) if n == 0), None)
        blocked = next((d['to_target'] for d in solved.diagnostics if d.get('reason') == 'transition_blocked'), None)
        progress = failed if failed is not None else blocked if blocked is not None else len(targets)
        history.append(dict(metadata, complete=bool(solved.configurations),
            first_infeasible_target=failed,
            failed_target_details=solved.target_diagnostics[failed] if failed is not None else None,
            transition_diagnostics=list(solved.diagnostics)))
        return progress, solved, arrays, dict(metadata), len(history)-1

    for _, arrays, metadata in ordered[:int(max_attempts)]:
        attempt = evaluate(arrays, metadata)
        if best is None or attempt[0] > best[0] or attempt[1].configurations:
            best = attempt
        if attempt[1].configurations:
            break
    section_diagnostics = []
    if connect_sections and best is not None and not best[1].configurations:
        from .mobile_sections import plan_mobile_sections
        joined = plan_mobile_sections(targets, ordered, arm_in_base, height=height,
            section_size=section_size, proposal_limit=section_proposals,
            beam_width=section_beam_width, **options)
        section_diagnostics = list(joined.diagnostics)
        for diagnostic in section_diagnostics:
            checked_any.update(diagnostic.get('checked_target_indices',[]))
        if joined.configurations:
            checked_any.update(range(len(targets)))
            history.append(dict(complete=True, strategy='connected_sections',
                                section_diagnostics=section_diagnostics))
            best = (len(targets),joined,best[2],best[3],len(history)-1)
    # A smooth compact-support displacement changes a neighbourhood, never
    # just one footprint. Every changed and unchanged TCP is then validated.
    # Continue from an improved path, retaining prior successful adjustments.
    moves = [(x,y,None) for size in (.15,.3)
             for x,y in ((size,0),(-size,0),(0,size),(0,-size))]
    moves += [(x,y,span) for span in (100,50,25,10) for size in (.025,.05,.1,.15,.3)
              for x,y in ((size,0),(-size,0),(0,size),(0,-size))]
    move_index = 0
    for _ in range(int(repair_attempts)):
        if best is None or best[1].configurations:
            break
        index, _, arrays, metadata, _ = best
        index = min(index,len(targets)-1)
        if move_index >= len(moves):
            break  # No improvement: do not repeat identical failed repairs.
        dx,dy,span = moves[move_index]
        move_index += 1
        span = max(50,int(metadata['window'])) if span is None else span
        u = np.abs(np.arange(len(targets))-index)/span
        blend = np.where(u < 1, .5*(1+np.cos(np.pi*np.minimum(u,1))), 0.)
        positions, direction, tangent = arrays
        changed = positions+blend[:,None]*(dx*direction+dy*tangent)
        mount = as_plane(arm_in_base).origin
        arm = changed+mount[0]*direction+mount[1]*tangent
        relative = np.array([as_plane(t).origin[:2] for t in targets])-arm
        normals = np.array([as_plane(t).zaxis[:2] for t in targets])
        bad = np.flatnonzero((np.linalg.norm(relative,axis=1)>1.75+1e-10) |
                             (np.einsum('ij,ij->i',relative,normals)<=0))
        repair = dict(target=index, span=span, robot_x_shift=dx, robot_y_shift=dy)
        if len(bad):
            history.append(dict(local_repair=repair, complete=False,
                reason='placement_geometry', first_infeasible_target=int(bad[0])))
            continue
        repaired_meta = dict(metadata, local_repairs=metadata.get('local_repairs',[])+[repair])
        repaired_meta['max_arm_xy_reach'] = float(np.linalg.norm(relative,axis=1).max())
        repaired_meta['xy_second_difference'] = (float(np.mean(np.linalg.norm(np.diff(changed,n=2,axis=0),axis=1)))
                                                 if len(targets)>2 else 0.)
        repaired_meta['proposal_smoothness_score'] = repaired_meta.pop('smoothness_score',
            repaired_meta.get('proposal_smoothness_score'))
        attempt = evaluate((changed,direction,tangent),repaired_meta)
        if attempt[0] > best[0] or attempt[1].configurations:
            best = attempt
            move_index = 0
    if best is not None:
        result = best[1]
    result.diagnostics.append(dict(mode='smooth_offset', attempts=history,
        section_diagnostics=section_diagnostics, geometry_rejected_count=len(rejected), geometry_rejection_examples=rejected[:5],
        proposal_count=len(proposals), attempt_limit=int(max_attempts),
        selected=history[best[4]] if best is not None and result.configurations else None,
        best_attempt=best[4] if best is not None else None,
        checked_in_any_attempt=len(checked_any), repair_attempt_limit=int(repair_attempts),
        summary_scope='target counts describe the best attempt, not the last attempt',
        reason='complete' if result.configurations else 'smooth_proposals_exhausted',
        scope='bounded smooth proposals; failure does not prove global infeasibility',
        total_seconds=perf_counter()-started))
    return result

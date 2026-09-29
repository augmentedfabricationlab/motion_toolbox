"""Smooth offset repairs; feasibility is delegated to the existing validator."""
from time import perf_counter
import numpy as np
from .geometry import Plane
from .recording import event, metric, recorded
from .xy_smoothing import significant_reversals


def failure_indices(result):
    indices = list(result['unreachable_points'])
    indices.extend(d['to_target'] for d in result['transition_failures'])
    if result['disconnected_target'] is not None:
        indices.append(result['disconnected_target'])
    return sorted(set(indices))


def repair_weights(n, first, last, radius):
    """C2 quintic ramps, zero displacement/slope at unchanged join boundaries."""
    lo, hi = max(0, first-radius), min(n-1, last+radius)
    weights = np.zeros(n)
    weights[first:last+1] = 1.
    def ramp(t):
        return t*t*t*(10.+t*(-15.+6.*t))
    if first > lo:
        weights[lo:first] = ramp((np.arange(lo,first)-(first-radius))/radius)
    if hi > last:
        weights[last+1:hi+1] = ramp(((last+radius)-np.arange(last+1,hi+1))/radius)
    return weights, lo, hi


def repair_anchors(targets, first, last):
    """Retain spatial/height changes, orientation changes, extrema and reversals."""
    points = np.asarray([t.origin for t in targets])
    selected = {first, last}
    for axis in range(3):
        values = points[first:last+1, axis]
        selected.update((first+int(values.argmin()), first+int(values.argmax())))
        selected.update(first+int(i) for i in significant_reversals(values,.10))
    previous = first
    for i in range(first+1, last):
        if (i-previous >= 20 or np.linalg.norm(points[i,:2]-points[previous,:2]) >= .15
                or abs(points[i,2]-points[previous,2]) >= .15
                or np.dot(targets[i].zaxis, targets[previous].zaxis) < np.cos(np.deg2rad(10))):
            selected.add(i)
            previous = i
    return sorted(selected)


@recorded
def repair_offsets(targets, bases, initial, *, validate, probe, normal_offset,
                   tangent_offset, search_extent, cancel_check=None, build_frames=None,
                   prefer_progress=False, yaw_margin_degrees=0.):
    """Beam of connected full-path repairs with overlapping, expanding windows.

    The finite grid and retained alternatives are a search domain, not a proof
    that no continuous placement exists. No elapsed-time limit is imposed.
    With yaw enabled, internal profiles and the returned applied array have a
    third column in radians, relative to the regenerated geometric heading.
    """
    started = perf_counter()
    event('mobile.repair_policy', traversal='progress_first' if prefer_progress else 'breadth_first',
          retained_alternatives=4)
    n = len(targets)
    axes_x = np.asarray([b.xaxis for b in bases])
    axes_y = np.asarray([b.yaxis for b in bases])
    origins = np.asarray([b.origin for b in bases])
    preferred = np.array([normal_offset, abs(tangent_offset)])
    margin = np.deg2rad(float(yaw_margin_degrees))
    if not np.isfinite(margin) or not 0 <= margin <= np.pi:
        raise ValueError('yaw_margin_degrees must be between 0 and 180')
    if margin:
        preferred = np.r_[preferred, 0.]
    side = 1. if tangent_offset >= 0 else -1.
    attempts = []
    beam = [(np.zeros((n, len(preferred))), initial)]
    seen = {beam[0][0].tobytes()}
    def absolute_frames(absolute, indices):
        absolute = np.asarray(absolute)
        if build_frames is not None:
            result = build_frames(absolute[:,:2]*[1.,side], indices)
        else:
            q = absolute[:,:2]-preferred[:2]
            xyz = origins[indices]-q[:,0,None]*axes_x[indices]+side*q[:,1,None]*axes_y[indices]
            result = [Plane(o,x,y) for o,x,y in zip(xyz, axes_x[indices], axes_y[indices])]
        if margin:
            c,s = np.cos(absolute[:,2]),np.sin(absolute[:,2])
            result = [Plane(b.origin,cc*b.xaxis+ss*b.yaxis,-ss*b.xaxis+cc*b.yaxis)
                      for b,cc,ss in zip(result,c,s)]
        return result
    def frames(offsets):
        return absolute_frames(preferred+offsets, np.arange(n))
    def screen_pair(indices, absolute, offsets, first):
        corrections = offsets[indices]+absolute-preferred-offsets[first]
        absolute = preferred+corrections
        if margin and np.any(abs(absolute[:,2]) > margin+1e-12):
            return False
        proposed = absolute_frames(absolute, np.asarray(indices))
        return probe(indices, proposed)
    def cost(offsets):
        return float(np.mean(offsets**2)+10*np.sum(np.diff(offsets,axis=0)**2)
                     +100*np.sum(np.diff(offsets,n=2,axis=0)**2))
    def rank(item):
        offsets, result = item
        failed = failure_indices(result)
        return len(failed), -(failed[0] if failed else n), cost(offsets)
    best = beam[0]
    # Each round must strictly improve the best failure rank. Alternatives are
    # retained until their full sequence, including both joins, has been tested.
    while beam:
        next_beam = []
        pending = beam[1:] if prefer_progress else []
        active = beam[:1] if prefer_progress else beam
        for offsets, previous in active:
            failed = failure_indices(previous)
            if not failed:
                continue
            first = failed[0]
            last = first
            for i in failed[1:]:
                if i-last > 20:
                    break
                last = i
            center = preferred+offsets[first]
            screening = sorted(set(repair_anchors(targets,first,last)+[first,last]))
            grid = np.arange(0., search_extent+.05, .10)
            pairs = [np.array([d,t]) for d in grid if d > 0 for t in grid]
            pairs.sort(key=lambda p:(float(np.sum((p-center[:2])**2)), float(np.sum((p-preferred[:2])**2)), *p))
            if margin:
                # Try rotation at the current position first, then retain one
                # viable angle per XY candidate instead of multiplying the beam.
                pairs.insert(0, center[:2].copy())
                steps = np.arange(0.,margin,np.deg2rad(5))
                angles = sorted(set(np.clip(np.r_[steps,-steps,-margin,margin,center[2]],
                    -margin,margin)), key=lambda a:(abs(a-center[2]),abs(a),a))
            viable = []
            for pair in pairs:
                if cancel_check:
                    cancel_check()
                candidates = [np.r_[pair,a] for a in angles] if margin else [pair]
                for candidate in candidates:
                    if cancel_check:
                        cancel_check()
                    correction = candidate-preferred-offsets[first]
                    if np.linalg.norm(correction) < 1e-10:
                        continue
                    if screen_pair(screening, candidate, offsets, first):
                        viable.append(candidate)
                        break
                if len(viable) == 4:
                    break
            # Refine viable offset neighborhoods before expensive full-path work.
            for spacing in (.05, .025):
                refined = []
                for pair in viable:
                    steps = [np.array([a,b])*spacing for a in (-1,0,1) for b in (-1,0,1)]
                    if margin:
                        steps = [np.r_[q, np.deg2rad(50*spacing)*a] for q in steps for a in (-1,0,1)]
                    neighborhood = [pair+step for step in steps]
                    neighborhood.sort(key=lambda p:float(np.sum((p-preferred)**2)))
                    for p in neighborhood:
                        if p[0] <= 0 or p[1] < 0 or np.any(p[:2] > search_extent) or (margin and abs(p[2]) > margin+1e-12):
                            continue
                        if screen_pair(screening, p, offsets, first):
                            refined.append(p)
                            break
                # Keep the coarse alternatives until whole-path checks decide;
                # a cheaper center placement can have incompatible joins.
                unique = {tuple(np.round(p,12)):p for p in viable+refined}
                viable = sorted(unique.values(),key=lambda p:float(np.sum((p-preferred)**2)))
            radii = []
            radius = 16
            while radius < n:
                radii.append(radius)
                radius *= 2
            radii.append(n)
            for radius in radii:
                weights, lo, hi = repair_weights(n, first, last, radius)
                anchors = sorted(set(repair_anchors(targets, lo, hi)+[first,last]))
                # Other known failures outside this repair's plateau are for
                # later rounds. They must not veto an otherwise useful partial
                # repair; the complete sequence is still validated below.
                known = set(failed)
                anchors = [i for i in anchors if i not in known or first <= i <= last]
                for pair in viable:
                    if cancel_check:
                        cancel_check()
                    trial = offsets+weights[:,None]*(pair-preferred-offsets[first])
                    if np.any((preferred+trial)[:,:2] < 0) or (margin and np.any(abs(trial[:,2]) > margin+1e-12)):
                        continue
                    key = trial.tobytes()
                    if key in seen:
                        continue
                    seen.add(key)
                    proposed = frames(trial)
                    attempt = dict(interval=[lo,hi], failed_interval=[first,last],
                                   offset_at_failure=pair[:2].tolist(), refinement_metres=.025,
                                   anchors=anchors, smoothness_cost=cost(trial))
                    if margin:
                        attempt.update(yaw_adjustment_degrees=float(np.rad2deg(pair[2])),yaw_refinement_degrees=1.25)
                    if not probe(anchors, [proposed[i] for i in anchors]):
                        attempt['state'] = 'anchor_rejection'
                    else:
                        result = validate(proposed)
                        attempt.update(state='validated' if result['fabrication_validated'] else 'full_path_rejection',
                                       failures=failure_indices(result), status=result['status'])
                        item = trial, result
                        if rank(item) < rank(best):
                            best = item
                        if result['fabrication_validated']:
                            attempts.append(attempt)
                            event('mobile.offset_repair', **attempt)
                            metric('mobile.repair_seconds', perf_counter()-started, 's')
                            return result, preferred+trial, attempts
                        if rank(item) < rank((offsets, previous)):
                            next_beam.append(item)
                    attempts.append(attempt)
                    event('mobile.offset_repair', **attempt)
                if next_beam:
                    break
        # Progress-first traversal retains alternatives for a dead end instead
        # of expanding every beam entry before pursuing an improving sequence.
        beam = sorted(pending+next_beam, key=rank)[:4]
    metric('mobile.repair_seconds', perf_counter()-started, 's')
    return best[1], preferred+best[0], attempts

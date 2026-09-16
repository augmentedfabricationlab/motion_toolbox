"""Bounded search for overlapping mobile sections with validated smooth joins."""
import numpy as np
from .geometry import Plane, as_plane
from .base_planning import BasePlan, plan_mobile_base


class _TransitionCache:
    """Reuse exact endpoint checks within this fixed-scene section search."""
    def __init__(self, callback):
        self.callback, self.cache, self.last_failure = callback, {}, None
        self.hits = 0

    def check(self, q0,b0,q1,b1):
        key = (np.asarray(q0,dtype=float).tobytes(),b0.matrix.tobytes(),
               np.asarray(q1,dtype=float).tobytes(),b1.matrix.tobytes())
        if key not in self.cache:
            accepted = bool(self.callback(q0,b0,q1,b1))
            checker = getattr(getattr(self.callback,'func',self.callback),'__self__',None)
            reason = getattr(checker,'last_failure',None) if not accepted else None
            if len(self.cache) >= 100000:
                self.cache.clear()
            self.cache[key] = accepted,reason
        else:
            self.hits += 1
        accepted,self.last_failure = self.cache[key]
        return accepted


def _slice_options(options, start, end):
    sliced = dict(options, _stop_on_unreachable=True)
    has_start = options.get('start_base') is not None
    if start:
        sliced.update(start_base=None, current_pose=None)
    times = options.get('time_intervals')
    if times is not None:
        sliced['time_intervals'] = (times[:end] if has_start and start == 0 else
                                   times[start+int(has_start):end-1+int(has_start)])
    return sliced


def blend_sections(previous, incoming, start):
    """Cubic smoothstep overlap, using the shortest yaw arc at each target."""
    overlap = len(previous)-start
    if overlap < 2:
        raise ValueError('At least two overlapping targets required')
    combined = list(previous[:start])
    for j,b in enumerate(incoming):
        if j >= overlap:
            combined.append(b)
            continue
        a = previous[start+j]
        if j == 0:
            combined.append(a)
            continue
        if j == overlap-1:
            combined.append(b)
            continue
        t = j/(overlap-1)
        t = t*t*(3-2*t)
        yaw0 = np.arctan2(a.xaxis[1], a.xaxis[0])
        yaw1 = np.arctan2(b.xaxis[1], b.xaxis[0])
        yaw = yaw0+t*((yaw1-yaw0+np.pi)%(2*np.pi)-np.pi)
        combined.append(Plane((1-t)*a.origin+t*b.origin,
                              (np.cos(yaw),np.sin(yaw),0),(-np.sin(yaw),np.cos(yaw),0)))
    return combined


def plan_mobile_sections(targets, proposals, arm_in_base, *, height=0., section_size=100,
                         proposal_limit=6, beam_width=2, **options):
    """Validate sections independently, then validate each entire joined prefix.

    Independent IK branches are never concatenated: the joined prefix graph
    chooses mutually compatible arm configurations, including swept checks.
    Only complete, validated paths populate the returned output planes.
    """
    if int(section_size) != section_size or section_size < 4:
        raise ValueError('section_size must be an integer >= 4')
    if int(proposal_limit) != proposal_limit or proposal_limit < 1:
        raise ValueError('section_proposals must be a positive integer')
    if int(beam_width) != beam_width or beam_width < 1:
        raise ValueError('section_beam_width must be a positive integer')
    n = len(targets)
    section_size = int(section_size)
    overlap = max(2,section_size//2)
    options = dict(options)
    options.setdefault('_candidate_cache', {})
    transition_cache = None
    if options.get('transition_check') is not None:
        transition_cache = _TransitionCache(options['transition_check'])
        options['transition_check'] = transition_cache.check
    library = []
    for _,arrays,metadata in proposals[:int(proposal_limit)]:
        library.append(([Plane((p[0],p[1],height),(*d,0),(*t,0)) for p,d,t in zip(*arrays)], metadata))
    beam, logs, checked = [], [], set()
    end, start = min(n,section_size), 0
    mount = as_plane(arm_in_base).origin
    while True:
        alternatives = []
        report = dict(start_target=start,end_target=end-1,valid_sections=0,joins_tested=0,rejections=[])
        for proposal_index,(planes,metadata) in enumerate(library):
            section = plan_mobile_base(targets[start:end], [[b] for b in planes[start:end]],
                                       **_slice_options(options,start,end))
            checked.update(start+i for i,c in enumerate(section.candidate_counts) if c is not None)
            if not section.configurations:
                bad = next((i for i,c in enumerate(section.candidate_counts) if c == 0),None)
                report['rejections'].append(dict(proposal=proposal_index,phase='section',
                    failed_target=start+bad if bad is not None else None,
                    target_details=section.target_diagnostics[bad] if bad is not None else None,
                    transitions=section.diagnostics,transition_index_offset=start))
                continue
            report['valid_sections'] += 1
            prefixes = beam if start else [(None,None,[])]
            for previous,_,joins in prefixes:
                combined = blend_sections(previous,planes[start:end],start) if start else planes[:end]
                # Interpolation must retain placement constraints even for
                # numeric callers without a robot adapter's base_valid callback.
                valid = True
                for target,base in zip(targets[:end],combined):
                    arm = base.origin+mount[0]*base.xaxis+mount[1]*base.yaxis
                    delta = as_plane(target).origin[:2]-arm[:2]
                    if np.linalg.norm(delta)>1.75+1e-10 or delta @ as_plane(target).zaxis[:2] <= 0:
                        valid = False
                        break
                if not valid:
                    report['rejections'].append(dict(proposal=proposal_index,phase='join',reason='placement_geometry'))
                    continue
                report['joins_tested'] += int(bool(start))
                joined = (plan_mobile_base(targets[:end],[[b] for b in combined],
                                          **_slice_options(options,0,end)) if start else section)
                if not joined.configurations:
                    bad = next((i for i,c in enumerate(joined.candidate_counts) if c == 0),None)
                    report['rejections'].append(dict(proposal=proposal_index,phase='join',
                        failed_target=bad,
                        target_details=joined.target_diagnostics[bad] if bad is not None else None,
                        transitions=joined.diagnostics))
                    continue
                trace = joins+[dict(start_target=start,end_target=end-1,proposal=proposal_index,
                    overlap_end=len(previous)-1 if start else None,settings=metadata)]
                alternatives.append((combined,joined,trace))
        logs.append(report)
        if not alternatives:
            return BasePlan([],[],float('inf'),[None]*n,
                [dict(mode='sections',reason='section_connection_failed',sections=logs,
                      transition_cache_hits=transition_cache.hits if transition_cache else 0,
                      checked_target_indices=sorted(checked),
                      validated_prefix_targets=len(beam[0][0]) if beam else 0,
                      scope='bounded section/beam search; no complete path validated')])
        # Prefer smooth joins, not minimal total base travel.
        def roughness(item):
            poses = item[0]
            xy = np.array([b.origin[:2] for b in poses])
            yaw = np.unwrap([np.arctan2(b.xaxis[1],b.xaxis[0]) for b in poses])
            return float(np.linalg.norm(np.diff(xy,n=2,axis=0),axis=1).sum()+abs(np.diff(yaw,n=2)).sum())
        beam = sorted(alternatives,key=roughness)[:int(beam_width)]
        if end == n:
            result = beam[0][1]
            result.diagnostics.append(dict(mode='sections',reason='complete',sections=logs,joins=beam[0][2],
                                           transition_cache_hits=transition_cache.hits if transition_cache else 0,
                                           checked_target_indices=sorted(checked)))
            return result
        start = end-overlap
        end = min(n,start+section_size)

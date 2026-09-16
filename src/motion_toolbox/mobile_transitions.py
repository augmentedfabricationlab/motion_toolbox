"""Shared mobile transition checks and explanations (metres, radians, seconds)."""
from .runtime import check_deadline
import math
import numpy as np


class MobileTransitions:
    def __init__(self, options):
        self.options = options
        self.cache = {}
        self.previous = None
        self.reset()

    def reset(self):
        check_deadline('mobile_transitions.reset')
        self.rejections = {}
        self.examples = {}

    def reject(self, name, count=1, **example):
        check_deadline('mobile_transitions.reject')
        self.rejections[name] = self.rejections.get(name, 0)+int(count)
        self.examples.setdefault(name, example)

    def reachable(self, i, previous, q, base):
        """Return valid predecessor indices; count the first violated constraint.

        Cheap bounds are evaluated in arrays before any swept collision query.
        All admissible predecessors are returned; callers may stop after one.
        """
        check_deadline('mobile_transitions.reachable')
        o = self.options
        if previous is not self.previous:
            self.previous = previous
            self.q0 = np.asarray([p[0] for p in previous], dtype=float)
            self.origins = np.asarray([p[1].origin for p in previous])
            self.yaw0 = np.array([math.atan2(p[1].xaxis[1], p[1].xaxis[0]) for p in previous])
            self.base = None
        if base is not self.base:
            self.base = base
            self.distance = np.linalg.norm(self.origins-base.origin, axis=1)
            yaw1 = math.atan2(base.xaxis[1], base.xaxis[0])
            self.yaw = abs((yaw1-self.yaw0+math.pi) % (2*math.pi)-math.pi)
        q0, distance, yaw = self.q0, self.distance, self.yaw
        delta = np.asarray(q)-q0
        mask = np.asarray(o['periodic'] if o['periodic'] is not None else [False]*len(q), dtype=bool)
        delta[:, mask] = (delta[:, mask]+math.pi) % (2*math.pi)-math.pi
        delta = abs(delta)
        keep = np.ones(len(previous), dtype=bool)

        def bound(name, values, limit):
            check_deadline('mobile_transitions.bound')
            if limit is None:
                return
            exceeded = values > np.asarray(limit)
            bad = keep & (exceeded.any(axis=1) if exceeded.ndim == 2 else exceeded)
            if bad.any():
                row = int(np.flatnonzero(bad)[0])
                example = dict(measured=np.asarray(values[row]).tolist(), limit=np.asarray(limit).tolist())
                if exceeded.ndim == 2:
                    example['joint_indices'] = np.flatnonzero(exceeded[row]).tolist()
                self.reject(name, bad.sum(), **example)
                keep[bad] = False

        bound('base_step', distance, o['max_base_step'])
        bound('yaw_step', yaw, o['max_yaw_step'])
        bound('joint_step', delta, o['max_joint_step'])
        if o['time_intervals'] is not None:
            dt = o['time_intervals'][i if o['start_base'] is not None else i-1]
            bound('base_speed', distance/dt, o['max_base_speed'])
            bound('yaw_speed', yaw/dt, o['max_yaw_speed'])
            bound('joint_speed', delta/dt, o['max_joint_speed'])
        callback = o['transition_check']
        for a in np.flatnonzero(keep):
            check_deadline('mobile_transitions.reachable')
            if callback is not None:
                oldq, oldbase = previous[a]
                key = (np.asarray(oldq, dtype=float).tobytes(), oldbase.matrix.tobytes(),
                       np.asarray(q, dtype=float).tobytes(), base.matrix.tobytes())
                cached = self.cache.get(key)
                if cached is None:
                    valid = callback(oldq, oldbase, q, base)
                    checker = getattr(getattr(callback, 'func', callback), '__self__', None)
                    reason = getattr(checker, 'last_failure', None) if not valid else None
                    cached = bool(valid), str(reason) if reason else None
                    if len(self.cache) >= 100000:
                        self.cache.clear()
                    self.cache[key] = cached
                if not cached[0]:
                    self.reject('transition_collision' if cached[1] else 'transition_check', detail=cached[1])
                    continue
            yield int(a)

    def diagnostic(self, i):
        check_deadline('mobile_transitions.diagnostic')
        return dict(reason='transition_blocked', from_target=i-1 if i else 'start',
                    to_target=i, rejection_counts=dict(self.rejections),
                    constraint_examples=dict(self.examples),
                    counting='first violated constraint per tested pair; indices are zero-based')

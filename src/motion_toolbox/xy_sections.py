"""Prepared straight/arc geometry in metres, independent of robot feasibility."""
from time import perf_counter
import numpy as np
from .geometry import Plane
from .xy_smoothing import significant_reversals


DEFAULTS = dict(arc_turn_threshold_deg=45., arc_fit_rms=.03,
                arc_fit_max=.075, transition_length=.5, reversal_excursion=.5)


def settings(options=None):
    result = dict(DEFAULTS)
    if options is not None and not isinstance(options, dict):
        raise ValueError('geometry_options must be a dictionary')
    if options:
        if set(options)-set(result):
            raise ValueError('Unknown geometry options: '+str(set(options)-set(result)))
        result.update(options)
    for key, value in result.items():
        if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
            raise ValueError(key+' must be finite and positive')
        result[key] = float(value)
    if result['arc_turn_threshold_deg'] > 180 or result['arc_fit_max'] < result['arc_fit_rms']:
        raise ValueError('Require turn threshold <= 180 and arc_fit_max >= arc_fit_rms')
    return result


def _samples(points):
    """Equal spatial weighting; bounded fit size and no quadratic distance matrix."""
    points = np.asarray(points, dtype=float)
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 1e-10]
    if distance[-1] <= 1e-10:
        return points[:1]
    # Select existing samples nearest equal-distance stations. Interpolating
    # chords would shrink a perfect circle and bias its fitted center/radius.
    points, distance = points[keep], distance[keep]
    stations = np.linspace(0., distance[-1], min(129, len(points), max(5, int(distance[-1]/.05)+1)))
    right = np.minimum(np.searchsorted(distance, stations), len(points)-1)
    left = np.maximum(right-1, 0)
    indices = np.where(abs(distance[left]-stations) <= abs(distance[right]-stations), left, right)
    return points[np.unique(indices)]


def _circle(points):
    q = _samples(points)
    if len(q) < 5:
        return None
    origin = q.mean(axis=0)
    scale = np.max(np.linalg.norm(q-origin, axis=1))
    if scale <= 1e-10:
        return None
    u = (q-origin)/scale
    design = np.column_stack((2*u, np.ones(len(u))))
    if np.linalg.cond(design) > 1e7:
        return None
    center = origin+scale*np.linalg.lstsq(design, np.sum(u*u, axis=1), rcond=None)[0][:2]
    center = _shared_center([q], center)
    return _measure(points, center)


def _shared_center(groups, center):
    """Geometric least squares with a separate nuisance radius for every pass."""
    center = np.asarray(center, dtype=float).copy()
    for _ in range(12):
        rows, errors = [], []
        for points in groups:
            delta = center-points
            radius = np.linalg.norm(delta, axis=1)
            if np.any(radius < 1e-10):
                return center
            jac = delta/radius[:, None]
            # Eliminate each pass radius; give each pass equal total weight.
            rows.append((jac-jac.mean(axis=0))/np.sqrt(len(points)))
            errors.append((radius-radius.mean())/np.sqrt(len(points)))
        jac, residual = np.vstack(rows), np.concatenate(errors)
        if np.linalg.cond(jac) > 1e7:
            break
        step = np.linalg.lstsq(jac, residual, rcond=None)[0]
        if not np.isfinite(step).all() or np.linalg.norm(step) > 10*max(1., np.linalg.norm(center)):
            break
        center -= step
        if np.linalg.norm(step) < 1e-9:
            break
    return center


def _measure(points, center):
    delta = np.asarray(points)-center
    radii = np.linalg.norm(delta, axis=1)
    radius = float(np.mean(np.linalg.norm(_samples(points)-center, axis=1)))
    residual = radii-radius
    angles = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
    return dict(center=np.asarray(center), radius=radius,
                rms=float(np.sqrt(np.mean(residual**2))),
                max_error=float(np.max(abs(residual))),
                turn_deg=float(np.rad2deg(np.ptp(angles))), residual=residual)


def _heading_turn(points):
    q = _samples(points)
    if len(q) < 3:
        return 0.
    directions = np.diff(q, axis=0)
    headings = np.unwrap(np.arctan2(directions[:, 1], directions[:, 0]))
    return float(np.degrees(np.ptp(headings)))


def _acceptable(fit, opts):
    return fit is not None and fit['rms'] <= opts['arc_fit_rms'] and fit['max_error'] <= opts['arc_fit_max']


def classify_sections(raw, smooth, longitudinal, *, options=None):
    """Classify inclusive fit intervals; ownership is unique in target order.

    Strong-curvature evidence propagates to subdivisions. A failed circle fit
    must not become a collection of accidentally 'straight' short arc pieces.
    """
    opts = settings(options)
    raw, smooth = np.asarray(raw, dtype=float), np.asarray(smooth, dtype=float)
    if any(p.ndim != 2 or p.shape[1] not in (2,3) or len(p)<2 or not np.isfinite(p).all() for p in (raw,smooth)):
        raise ValueError('Require at least two finite XY/XYZ points')
    if len(raw)!=len(smooth) or np.asarray(longitudinal).shape!=(len(raw),):
        raise ValueError('Raw, smoothed and longitudinal samples must correspond')
    raw, smooth = raw[:, :2], smooth[:, :2]
    n = len(raw)
    cuts = sorted(set([0]+significant_reversals(longitudinal, opts['reversal_excursion'])+[n-1]))
    sections = []

    def split(first, last, pass_id, strong_parent=False, depth=0):
        points = smooth[first:last+1]
        fit = _circle(points)
        turn = fit['turn_deg'] if fit is not None else _heading_turn(points)
        strong = turn >= opts['arc_turn_threshold_deg']
        heading_turn = _heading_turn(points)
        if heading_turn < 5. or (not strong and not strong_parent and heading_turn < opts['arc_turn_threshold_deg']):
            kind, measured = 'straight', fit
        else:
            measured = _circle(raw[first:last+1])
            if _acceptable(measured, opts) and (strong or strong_parent):
                kind = 'arc'
            elif not strong and not strong_parent and _acceptable(fit, opts):
                kind = 'straight'
            elif last-first >= 12 and depth < 12:
                # Splitting at the worst residual isolates a local change of
                # curvature, while a central guard prevents tiny noisy pieces.
                guard = max(5, (last-first)//4)
                residual = abs(fit['residual']) if fit is not None else np.zeros(len(points))
                middle = first+guard+int(np.argmax(residual[guard:len(points)-guard]))
                split(first, middle, pass_id, strong or strong_parent, depth+1)
                split(middle, last, pass_id, strong or strong_parent, depth+1)
                return
            else:
                kind = 'unresolved'
        section = dict(kind=kind, first=first, last=last, pass_id=pass_id,
                       heading_change_deg=turn)
        if measured is not None:
            section.update(center=measured['center'].tolist(), radius=measured['radius'],
                           fit_rms=measured['rms'], fit_max=measured['max_error'])
        sections.append(section)

    for pass_id, (first, last) in enumerate(zip(cuts, cuts[1:])):
        split(first, last, pass_id)
    if not sections:
        sections = [dict(kind='straight', first=0, last=n-1, pass_id=0, heading_change_deg=0.)]
    # Adjacent compatible arc pieces and repeated passes share one center, but
    # never one radius. Grouping is bounded by section count, not target count.
    groups = []
    for section in sections:
        if section['kind'] != 'arc':
            continue
        points = raw[section['first']:section['last']+1]
        for group in groups:
            seed = np.mean([s['center'] for s in group]+[section['center']], axis=0)
            if np.linalg.norm(np.asarray(group[0]['center'])-section['center']) > 4*opts['arc_fit_max']:
                continue
            spans = [raw[s['first']:s['last']+1] for s in group]+[points]
            center = _shared_center([_samples(p) for p in spans], seed)
            if all(_acceptable(_measure(p, center), opts) for p in spans):
                group.append(section)
                for s, p in zip(group, spans):
                    f = _measure(p, center)
                    s.update(center=center.tolist(), radius=f['radius'], fit_rms=f['rms'], fit_max=f['max_error'])
                break
        else:
            groups.append([section])
    for group_id, group in enumerate(groups):
        for section in group:
            section['arc_group'] = group_id
    ids = np.empty(n, dtype=int)
    for i, section in enumerate(sections):
        section['id'] = i
        ids[section['first']:section['last']+1] = i
    return sections, ids, opts


class SectionGeometry:
    """Reusable vectorized pose builder; signed offsets are absolute metres."""
    def __init__(self, points, legacy_frames, sections, section_ids, options):
        self.points = np.column_stack((np.asarray(points)[:, :2], np.zeros(len(points))))
        self.x = legacy_frames['x_axes']
        self.y = legacy_frames['y_axes']
        self.sections, self.section_ids, self.options = sections, section_ids, options
        self.transitions = []
        distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points[:, :2], axis=0), axis=1))]
        for left, right in zip(sections, sections[1:]):
            same = (left['kind'] == right['kind'] == 'straight' or
                    left.get('arc_group', -1) == right.get('arc_group', -2))
            if same:
                continue
            boundary = right['first']
            width = min(options['transition_length']/2,
                        (distance[boundary]-distance[left['first']])/2,
                        (distance[right['last']]-distance[boundary])/2)
            if width <= 1e-10:
                continue
            indices = np.flatnonzero(abs(distance-distance[boundary]) < width)
            u = (distance[indices]-distance[boundary]+width)/(2*width)
            self.transitions.append(dict(left=left['id'], right=right['id'],
                indices=indices, weights=u**3*(10+u*(-15+6*u)), length=2*width))

    def _pose(self, ids, offsets, section):
        if section['kind'] != 'arc':
            return (self.points[ids]-offsets[:, :1]*self.x[ids]+offsets[:, 1:]*self.y[ids], self.x[ids])
        center = np.asarray(section['center'])
        delta = self.points[ids, :2]-center
        radii = np.linalg.norm(delta, axis=1)
        enlarged = radii+offsets[:, 0]
        if np.any(radii <= 1e-10) or np.any(enlarged <= 1e-10):
            raise ValueError('Arc offset has nonpositive radius')
        # +Y = up cross inward X is clockwise around the fitted center.
        theta = np.arctan2(delta[:, 1], delta[:, 0])-offsets[:, 1]/enlarged
        radial = np.column_stack((np.cos(theta), np.sin(theta), np.zeros(len(ids))))
        origin = np.column_stack((np.tile(center, (len(ids), 1)), np.zeros(len(ids))))+enlarged[:, None]*radial
        return origin, -radial

    def frames(self, offsets, indices=None):
        ids = np.arange(len(self.points)) if indices is None else np.asarray(indices, dtype=int)
        offsets = np.broadcast_to(np.asarray(offsets, dtype=float), (len(ids), 2))
        if not np.isfinite(offsets).all():
            raise ValueError('Offsets must be finite')
        origins, x = np.empty((len(ids), 3)), np.empty((len(ids), 3))
        for sid in np.unique(self.section_ids[ids]):
            selected = self.section_ids[ids] == sid
            origins[selected], x[selected] = self._pose(ids[selected], offsets[selected], self.sections[sid])
        for transition in self.transitions:
            selected = np.isin(ids, transition['indices'])
            if not np.any(selected):
                continue
            ii, oo = ids[selected], offsets[selected]
            weight = transition['weights'][np.searchsorted(transition['indices'], ii)]
            a, ax = self._pose(ii, oo, self.sections[transition['left']])
            b, bx = self._pose(ii, oo, self.sections[transition['right']])
            origins[selected] = a+(b-a)*weight[:, None]
            yaw = np.arctan2(ax[:, 1], ax[:, 0])
            delta = (np.arctan2(bx[:, 1], bx[:, 0])-yaw+np.pi)%(2*np.pi)-np.pi
            yaw += weight*delta
            x[selected] = np.column_stack((np.cos(yaw), np.sin(yaw), np.zeros(len(ii))))
        y = np.column_stack((-x[:, 1], x[:, 0], np.zeros(len(ids))))
        return [Plane(o, xx, yy) for o, xx, yy in zip(origins, x, y)]


def prepare_sections(raw, smooth, guide, legacy_frames, *, mode='auto', options=None):
    started = perf_counter()
    if mode not in ('auto', 'legacy'):
        raise ValueError('geometry_mode must be auto or legacy')
    opts = settings(options)
    if mode == 'legacy':
        sections = [dict(id=0, kind='straight', first=0, last=len(raw)-1, pass_id=0, heading_change_deg=0.)]
        ids = np.zeros(len(raw), dtype=int)
    else:
        sections, ids, opts = classify_sections(raw, smooth, guide['longitudinal'], options=opts)
    geometry = SectionGeometry(smooth, legacy_frames, sections, ids, opts)
    return geometry, dict(geometry_mode=mode, geometry_options=opts, path_sections=sections,
        section_ids=ids, transition_regions=geometry.transitions,
        unresolved_sections=[s['id'] for s in sections if s['kind'] == 'unresolved'],
        classification_seconds=perf_counter()-started)

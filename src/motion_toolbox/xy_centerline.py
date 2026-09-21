"""Collapse repeated XY traversals to one spatial centerline, without offsets."""
import numpy as np
from .recording import recorded


@recorded
def centerline_xy(points, stations=201, bins=100, smoothing_fraction=.08):
    """Average only perpendicular to the dominant XY principal axis.

    PCA supplies longitudinal s and transverse t coordinates. Spatial bins of s
    first average t, then a Gaussian local-linear fit smooths those bin means.
    Nonempty bins get equal weight, limiting bias from dwell/sample density.
    Gaussian bandwidth is smoothing_fraction times the full longitudinal span.
    The output spans exactly min(s)..max(s); longitudinal coordinates are never
    averaged or shortened. mapped_points retain each input's s and original order.

    This is a single-valued t(s) model for broad open walls. Folded, branching or
    closed shapes may not have one meaningful centerline in this representation.
    No target orientation, robot model, placement or reachability is used.
    """
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] not in (2, 3) or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError('Provide at least two finite XY or XYZ positions')
    for name, value in [('stations', stations), ('bins', bins)]:
        if isinstance(value, bool) or int(value) != value or value < 2:
            raise ValueError(name+' must be an integer >= 2')
    if not np.isfinite(smoothing_fraction) or smoothing_fraction <= 0:
        raise ValueError('smoothing_fraction must be positive and finite')
    p = p[:, :2]
    origin = p.mean(axis=0)
    _, singular, axes = np.linalg.svd(p-origin, full_matrices=False)
    direction = axes[0].copy()
    if direction[np.argmax(abs(direction))] < 0:
        direction *= -1
    perpendicular = np.array([-direction[1], direction[0]])
    s, t = (p-origin)@direction, (p-origin)@perpendicular
    low, high = float(s.min()), float(s.max())
    span = high-low
    if span <= 64*np.finfo(float).eps*max(1., float(np.max(abs(p)))):
        raise ValueError('No nonzero longitudinal extent')
    labels = np.minimum(((s-low)/span*int(bins)).astype(int), int(bins)-1)
    count = np.bincount(labels, minlength=int(bins))
    occupied = count > 0
    bs = np.bincount(labels, weights=s, minlength=int(bins))[occupied]/count[occupied]
    bt = np.bincount(labels, weights=t, minlength=int(bins))[occupied]/count[occupied]
    ss = np.linspace(low, high, int(stations))
    transverse = []
    bandwidth = smoothing_fraction*span
    for station in ss:
        delta = (bs-station)/bandwidth
        log_weight = -.5*delta**2
        weight = np.exp(log_weight-log_weight.max())
        design = np.column_stack((np.ones(len(bs)), delta))
        coefficients = np.linalg.lstsq(design*np.sqrt(weight[:, None]), bt*np.sqrt(weight), rcond=None)[0]
        transverse.append(coefficients[0])
    transverse = np.asarray(transverse)
    curve = origin+ss[:, None]*direction+transverse[:, None]*perpendicular
    mapped = origin+s[:, None]*direction+np.interp(s, ss, transverse)[:, None]*perpendicular
    return dict(curve=curve, mapped_points=mapped, direction=direction,
                perpendicular=perpendicular, origin=origin, longitudinal=s,
                transverse=t, station_longitudinal=ss, station_transverse=transverse,
                longitudinal_extent=span, bandwidth=bandwidth,
                principal_variance_fraction=float(singular[0]**2/np.sum(singular**2)),
                bins=int(bins), stations=int(stations), smoothing_fraction=float(smoothing_fraction))

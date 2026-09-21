"""Ordered XY curve smoothing with a per-target deviation bound; no robot model."""
import numpy as np
from .recording import recorded

from .xy_averaging import moving_average_xy, line_length


@recorded
def smooth_xy(points, max_deviation, curvature_weight=100., max_iterations=60000,
              tolerance=1e-5):
    """Minimize squared steps + weighted squared second differences inside disks.

    Each output i stays within max_deviation of input i in XY. Large excursions
    therefore survive even if the endpoints coincide. The convex whole-path
    objective anticipates reversals without stitching independently fitted passes.
    Height is ignored. Endpoints may move within their disks. Input index is the
    parameter, not elapsed time; uneven sampling retains its original weighting.

    Accelerated projected gradient uses the bound 4 + 16*curvature_weight on the
    gradient Lipschitz constant. Convergence uses a convex objective-gap upper bound, normalized by the
    objective (with a floor of one in deviation-normalized coordinates).
    """
    raw = moving_average_xy(points, 1)
    if not np.isfinite(max_deviation) or max_deviation <= 0:
        raise ValueError('max_deviation must be finite and positive, in input units')
    if not np.isfinite(curvature_weight) or curvature_weight < 0:
        raise ValueError('curvature_weight must be finite and nonnegative')
    if isinstance(max_iterations, bool) or int(max_iterations) != max_iterations or max_iterations < 1:
        raise ValueError('max_iterations must be a positive integer')
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('tolerance must be finite and positive')
    center = raw[0].copy()
    p = (raw-center)/max_deviation
    lipschitz = 4.+16.*curvature_weight

    def gradient(x):
        d = np.diff(x, axis=0)
        g = np.zeros_like(x)
        g[:-1] -= d
        g[1:] += d
        dd = np.diff(d, axis=0)*curvature_weight
        g[:-2] += dd
        g[1:-1] -= 2.*dd
        g[2:] += dd
        return g

    def project(x):
        d = x-p
        return p+d/np.maximum(1., np.linalg.norm(d, axis=1))[:, None]

    x, y, momentum = p.copy(), p.copy(), 1.
    converged = False
    for iteration in range(1, int(max_iterations)+1):
        nxt = project(y-gradient(y)/lipschitz)
        if np.sum((y-nxt)*(nxt-x)) > 0.:
            momentum = 1.
        next_momentum = (1.+np.sqrt(1.+4.*momentum**2))/2.
        y = nxt+(momentum-1.)/next_momentum*(nxt-x)
        x, momentum = nxt, next_momentum
        if iteration % 100 == 0 or iteration == max_iterations:
            g = gradient(x)
            residual = float(np.max(np.linalg.norm(x-project(x-g/lipschitz), axis=1)))
            objective = .5*(np.sum(np.diff(x, axis=0)**2)+curvature_weight*np.sum(np.diff(x, n=2, axis=0)**2))
            gap = max(0., float(np.sum(g*(x-p))+np.linalg.norm(g, axis=1).sum()))
            if gap <= tolerance*max(1., objective):
                converged = True
                break
    curve = center+max_deviation*x
    deviations = np.linalg.norm(curve-raw, axis=1)
    return dict(curve=curve, max_deviation=float(max_deviation),
                measured_max_deviation=float(deviations.max()), deviations=deviations,
                length=line_length(curve), raw_length=line_length(raw),
                curvature_weight=float(curvature_weight), iterations=iteration,
                converged=converged, projected_step_residual=residual,
                objective_gap_bound=gap, normalized_objective=float(objective),
                step_energy=float(np.sum(np.diff(curve, axis=0)**2)),
                curvature_energy=float(np.sum(np.diff(curve, n=2, axis=0)**2)))


@recorded
def significant_reversals(values, excursion):
    """Hysteresis extrema: only confirm a turn after the specified excursion.

    This diagnostic does not control the smoother. The final unconfirmed extremum
    is omitted. A PCA coordinate is useful for broad open walls, but not intrinsic
    wall arc length and not a general detector for closed/folded walls.
    """
    values = np.asarray(values, dtype=float)
    if values.ndim != 1 or not len(values) or not np.isfinite(values).all():
        raise ValueError('Provide finite one-dimensional values')
    if not np.isfinite(excursion) or excursion <= 0:
        raise ValueError('excursion must be positive and finite')
    low = high = 0
    direction = 0
    turns = []
    for i, value in enumerate(values):
        if direction == 0:
            if value < values[low]:
                low = i
            if value > values[high]:
                high = i
            if value-values[low] >= excursion:
                direction, high = 1, i
            elif values[high]-value >= excursion:
                direction, low = -1, i
        elif direction > 0:
            if value > values[high]:
                high = i
            elif values[high]-value >= excursion:
                turns.append(high)
                direction, low = -1, i
        else:
            if value < values[low]:
                low = i
            elif value-values[low] >= excursion:
                turns.append(low)
                direction, high = 1, i
    return turns

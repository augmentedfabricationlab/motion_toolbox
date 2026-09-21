"""Compare position-only XY moving averages; no robot or base planning."""
import numpy as np
from .recording import recorded


@recorded
def moving_average_xy(points, window):
    """Centered box average at each original index, ignoring height.

    Truncate windows at the ends and renormalize available weights. Even
    windows average their two half-index alignments (half-weight endpoints),
    avoiding a half-sample phase shift. Windows are measured in input points.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3) or not len(points):
        raise ValueError('Provide a nonempty sequence of XY or XYZ positions')
    if not np.isfinite(points).all():
        raise ValueError('Positions must be finite')
    if isinstance(window, bool) or int(window) != window or not 1 <= window <= len(points):
        raise ValueError('Window must be an integer from 1 through the point count')
    window = int(window)
    kernel = np.ones(window if window % 2 else window + 1)
    if window % 2 == 0:
        kernel[[0, -1]] = .5
    start = len(kernel) // 2
    def convolve(values):
        return np.convolve(values, kernel, mode='full')[start:start + len(points)]
    weights = convolve(np.ones(len(points)))
    return np.column_stack([convolve(points[:, j]) / weights for j in (0, 1)])


def line_length(points):
    return float(np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1).sum())


@recorded
def compare_windows(points, windows=None, metric='length_ratio'):
    """Rank XY averages by length / their own endpoint distance, or length.

    Neither metric is a curvature or frequency guarantee. Coincident endpoints
    make the ratio undefined for a nonconstant line; those candidates score inf.
    No deduplication/resampling: repeated points retain their input weight.
    """
    points = np.asarray(points, dtype=float)
    raw = moving_average_xy(points, 1)
    if windows is None:
        windows = range(10 if len(points) >= 10 else 1, min(200, len(points)) + 1)
    windows = list(windows)
    if not windows:
        raise ValueError('Provide at least one averaging window')
    curves = [moving_average_xy(raw, w) for w in windows]
    order = np.argsort(windows, kind='stable')
    windows = [int(windows[i]) for i in order]
    if len(set(windows)) != len(windows):
        raise ValueError('Window sizes must be distinct')
    curves = [curves[i] for i in order]
    lengths = [line_length(c) for c in curves]
    chords = [float(np.linalg.norm(c[-1]-c[0])) for c in curves]
    tolerance = 64*np.finfo(float).eps*max(1., float(np.max(np.abs(raw))))
    ratios = [length/chord if chord > tolerance else 1. if length <= tolerance else float('inf')
              for length,chord in zip(lengths,chords)]
    if metric not in ('length', 'length_ratio'):
        raise ValueError('Metric must be length or length_ratio')
    scores = ratios if metric == 'length_ratio' else lengths
    if not np.isfinite(scores).any():
        raise ValueError('Length ratio is undefined for coincident endpoints; use metric=length')
    best = int(np.argmin(scores))
    shortest = int(np.argmin(lengths))
    minima = [windows[i] for i in range(1, len(windows)-1)
              if lengths[i] < lengths[i-1] and lengths[i] <= lengths[i+1]]
    return dict(windows=windows, curves=curves, lengths=lengths,
                metric=metric,scores=scores,chord_lengths=chords,length_ratios=ratios,
                best_index=best, best_window=windows[best], best_curve=curves[best],
                best_length=lengths[best],best_score=scores[best],
                shortest_index=shortest,shortest_window=windows[shortest],shortest_length=lengths[shortest],
                raw_xy=raw, raw_length=line_length(raw),
                local_minimum_windows=minima,
                endpoint_shifts=np.linalg.norm(curves[best][[0, -1]]-raw[[0, -1]], axis=1))

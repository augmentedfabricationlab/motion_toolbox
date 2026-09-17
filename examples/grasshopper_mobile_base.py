"""Rhino 8: XY averaging only; historical filepath retained.
Required: target_planes (List, Plane).
Optional: window_sizes (List, int; defaults 10..200), toolbox_src (Item, str),
          selection_metric (Item, str; length_ratio or length, default length_ratio).
          max_xy_deviation (Item, positive number in model units; enables bounded smoothing).
Outputs: averaged_line, comparison_lines, window_sizes_used, line_lengths,
         best_window, selected_length, selection_scores, shortest_window,
         shortest_length, projected_points, result, status.
Input model units are retained. No robot or base/arm planning is performed.
"""
import sys
import importlib
from pathlib import Path

# Clear former planner outputs on the existing filepath-loaded component.
base_planes, base_result, configurations, unreachable_points = [], [], [], []
joint_plan, path_cost = None, None
diagnostics, timings = [], {}
averaged_line, result = None, None
comparison_lines, projected_points, window_sizes_used, line_lengths = [], [], [], []
best_window, shortest_length = None, None
selected_length, shortest_window, selection_scores = None, None, []
status = ''
version = '0.1.23'
try:
    source = globals().get('toolbox_src')
    if source is None:
        source = str(Path(__file__).resolve().parents[1] / 'src')
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))
    import motion_toolbox.xy_averaging as averaging
    importlib.reload(averaging)
    targets = globals().get('target_planes')
    if targets is None or not len(targets):
        raise ValueError('Connect target_planes with List access')
    from motion_toolbox.geometry import as_plane
    positions = [as_plane(p).origin for p in targets]
    windows = globals().get('window_sizes')
    result = averaging.compare_windows(positions, windows if windows is not None and len(windows) else None,
                                       metric=('length' if globals().get('max_xy_deviation') is not None else
                                               globals().get('selection_metric') or 'length_ratio'))
    import Rhino.Geometry as rg
    def polyline(xy):
        return rg.PolylineCurve([rg.Point3d(float(x), float(y), 0.) for x, y in xy])
    comparison_lines = [polyline(c) for c in result['curves']]
    averaged_line = comparison_lines[result['best_index']]
    projected_points = [rg.Point3d(float(x), float(y), 0.) for x, y in result['raw_xy']]
    window_sizes_used, line_lengths = result['windows'], result['lengths']
    best_window, selected_length = result['best_window'], result['best_length']
    shortest_window, shortest_length = result['shortest_window'], result['shortest_length']
    selection_scores = result['scores']
    status = 'XY average only: window {} points; {} score {:.6g}; length {:.6g} model units.'.format(
        best_window,result['metric'],result['best_score'],selected_length)
    deviation = globals().get('max_xy_deviation')
    if deviation is not None:
        import motion_toolbox.xy_smoothing as smoothing
        importlib.reload(smoothing)
        smoothing_result = smoothing.smooth_xy(positions, float(deviation))
        result['smoothing'] = smoothing_result
        averaged_line = polyline(smoothing_result['curve'])
        selected_length = smoothing_result['length']
        status = 'Bounded XY smoothing: deviation {:.6g}; length {:.6g}; converged {}. Geometry only.'.format(
            smoothing_result['measured_max_deviation'], selected_length, smoothing_result['converged'])
except Exception as error:
    averaged_line, result = None, None
    comparison_lines, projected_points, window_sizes_used, line_lengths = [], [], [], []
    best_window, shortest_length = None, None
    selected_length, shortest_window, selection_scores = None, None, []
    status = '{}: {}'.format(type(error).__name__, error)

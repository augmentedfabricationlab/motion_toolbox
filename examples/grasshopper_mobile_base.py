"""Rhino 8: smooth passes and per-TCP geometric base frames.
Required: target_planes (List, Plane).
Optional: window_sizes (List, int; defaults 10..200), toolbox_src (Item, str),
          selection_metric (Item, str; length_ratio or length, default length_ratio).
          max_xy_deviation (Item, positive number in model units; enables bounded smoothing).
          create_base_planes (Item, bool; defaults true when max_xy_deviation supplied).
          units_to_metres (Item, positive float; otherwise read from Rhino document).
Outputs: base_planes, base_path, centerline, target_indices, averaged_line, comparison_lines, window_sizes_used, line_lengths,
         best_window, selected_length, selection_scores, shortest_window,
         shortest_length, projected_points, result, status.
Input model units are retained; offsets are 0.9 m normal and 1.2 m tangent.
Geometry only: no IK, collision or speed validation.
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
base_path, centerline, target_indices = None, None, []
status = ''
version = '0.1.26'
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
    if globals().get('create_base_planes', deviation is not None):
        if deviation is None:
            raise ValueError('Supply max_xy_deviation in model units to create base planes')
        import math
        import motion_toolbox.xy_centerline as centerlining
        import motion_toolbox.xy_offset as offsets
        importlib.reload(centerlining)
        importlib.reload(offsets)
        scale = globals().get('units_to_metres')
        if scale is None:
            import Rhino
            doc = Rhino.RhinoDoc.ActiveDoc
            if doc is None:
                raise ValueError('Supply units_to_metres when no Rhino document is active')
            scale = Rhino.RhinoMath.UnitScale(doc.ModelUnitSystem, Rhino.UnitSystem.Meters)
        scale = float(scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError('units_to_metres must be finite and positive')
        passes = smoothing_result['curve']
        guide = centerlining.centerline_xy(passes)
        numeric_targets = [as_plane(p) for p in targets]
        frames = offsets.centerline_offset_frames(
            guide['curve'], guide['mapped_points'],
            [p.xaxis for p in numeric_targets], [p.yaxis for p in numeric_targets],
            x_offset=-.9/scale, y_offset=1.2/scale, pass_points=passes)
        base_planes = [rg.Plane(rg.Point3d(*o), rg.Vector3d(*x), rg.Vector3d(*y))
                       for o,x,y in zip(frames['origins'],frames['x_axes'],frames['y_axes'])]
        base_path = polyline(frames['origins'][:, :2])
        centerline = polyline(guide['curve'])
        target_indices = list(range(len(targets)))
        result.update(base_frames=frames, centerline=guide, target_indices=target_indices,
                      units_to_metres=scale, fabrication_validated=False)
        status = 'Created {} per-TCP base planes: X toward wall, Y centerline tangent, Z global up. Smoothing converged: {}. Geometry only.'.format(
            len(base_planes), smoothing_result['converged'])
except Exception as error:
    base_planes, base_path, centerline, target_indices = [], None, None, []
    averaged_line, result = None, None
    comparison_lines, projected_points, window_sizes_used, line_lengths = [], [], [], []
    best_window, shortest_length = None, None
    selected_length, shortest_window, selection_scores = None, None, []
    status = '{}: {}'.format(type(error).__name__, error)

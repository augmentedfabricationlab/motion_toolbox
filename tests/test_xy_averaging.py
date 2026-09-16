import runpy
import sys
from pathlib import Path
from types import ModuleType
import numpy as np
import pytest
from motion_toolbox.xy_averaging import moving_average_xy, compare_windows
from motion_toolbox.geometry import Plane


def test_height_is_ignored_and_repeated_xy_stays_fixed():
    points = np.column_stack((np.ones(20), np.full(20, 2.), np.arange(20)))
    result = compare_windows(points)
    assert result['best_length'] == 0.
    for curve in result['curves']:
        np.testing.assert_allclose(curve, points[:, :2])


def test_centered_even_and_odd_windows_do_not_shift_linear_trend():
    points = np.column_stack((np.arange(20), np.arange(20)*2))
    for window in (3, 4, 5, 6):
        result = moving_average_xy(points, window)
        np.testing.assert_allclose(result[4:-4], points[4:-4])
        assert result.shape == points.shape
    assert moving_average_xy(points, 20).shape == points.shape
    np.testing.assert_allclose(moving_average_xy(points, 1), points)


def test_full_period_suppresses_square_wave_and_length_selects_it():
    points = np.column_stack((np.arange(200)*.01, np.tile(np.repeat([-1.,1.],10),10)))
    result = compare_windows(points, [10,20])
    assert result['best_window'] == 20
    assert len(result['best_curve']) == len(points)
    np.testing.assert_allclose(result['best_curve'][10:-10,1], 0., atol=1e-12)
    assert result['best_length'] == min(result['lengths'])


def test_ratio_uses_each_curve_endpoints_and_is_scale_invariant():
    i = np.arange(100)
    points = np.column_stack((i*.01, np.sin(i*.3)))
    result = compare_windows(points, [5,15,25])
    expected = [np.linalg.norm(np.diff(c,axis=0),axis=1).sum()/np.linalg.norm(c[-1]-c[0])
                for c in result['curves']]
    np.testing.assert_allclose(result['scores'], expected)
    assert result['best_index'] == int(np.argmin(expected))
    scaled = compare_windows(points*1000, [5,15,25])
    np.testing.assert_allclose(scaled['scores'], result['scores'])
    assert scaled['best_window'] == result['best_window']
    by_length = compare_windows(points, [5,15,25], metric='length')
    assert by_length['best_window'] == result['shortest_window']


def test_closed_nonconstant_line_does_not_get_a_finite_ratio():
    points = [[0.,0.],[1.,0.],[0.,0.]]
    with pytest.raises(ValueError, match='coincident endpoints'):
        compare_windows(points, [1])
    result = compare_windows(points, [1], metric='length')
    assert result['best_length'] == 2.
    assert result['length_ratios'] == [float('inf')]


@pytest.mark.parametrize('window', [0, 1.5, 21, True])
def test_invalid_windows_are_not_silently_coerced(window):
    with pytest.raises(ValueError):
        moving_average_xy(np.zeros((20,3)), window)


def test_actual_component_only_outputs_xy_curves_without_robot(monkeypatch):
    rhino, geometry = ModuleType('Rhino'), ModuleType('Rhino.Geometry')
    geometry.Point3d = lambda x,y,z: (x,y,z)
    geometry.PolylineCurve = lambda points: points
    rhino.Geometry = geometry
    monkeypatch.setitem(sys.modules, 'Rhino', rhino)
    monkeypatch.setitem(sys.modules, 'Rhino.Geometry', geometry)
    path = Path(__file__).resolve().parents[1]/'examples/grasshopper_mobile_base.py'
    targets = [Plane((i,i%2,100+i), (1,0,0),(0,1,0)) for i in range(30)]
    out = runpy.run_path(str(path),init_globals=dict(target_planes=targets,window_sizes=[5,10]))
    assert out['status'].startswith('XY average only'), out['status']
    assert len(out['averaged_line']) == 30
    assert all(p[2] == 0. for p in out['averaged_line'])
    assert not out['base_planes'] and not out['configurations'] and out['joint_plan'] is None
    assert len(out['comparison_lines']) == 2

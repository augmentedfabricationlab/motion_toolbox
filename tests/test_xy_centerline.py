import numpy as np
import pytest
from motion_toolbox.xy_centerline import centerline_xy


def test_repeated_passes_average_transverse_without_shortening():
    s = np.linspace(-2, 2, 301)
    passes = [np.column_stack((s, .12*s*s+offset)) for offset in [-.2, 0, .2]]
    points = np.vstack((passes[0], passes[1][::-1], passes[2]))
    result = centerline_xy(points)
    np.testing.assert_allclose(result['longitudinal_extent'], 4., atol=1e-12)
    np.testing.assert_allclose(result['curve'][[0,-1], 0], [-2,2], atol=1e-12)
    np.testing.assert_allclose(result['mapped_points'][:,0], points[:,0], atol=1e-12)
    assert np.max(abs(result['curve'][:,1]-.12*result['curve'][:,0]**2)) < .025


def test_rotation_translation_and_height_do_not_change_geometry():
    s = np.linspace(-2, 2, 200)
    p = np.column_stack((s,.1*s*s))
    rotation = np.array([[.6,-.8],[.8,.6]])
    a = centerline_xy(p)
    transformed = p@rotation+[10,20]
    b = centerline_xy(np.column_stack((transformed, np.sin(s))))
    np.testing.assert_allclose(b['mapped_points'], a['mapped_points']@rotation+[10,20], atol=1e-10)
    np.testing.assert_allclose(b['longitudinal_extent'], a['longitudinal_extent'], atol=1e-10)


def test_constant_xy_has_no_direction():
    with pytest.raises(ValueError, match='extent'):
        centerline_xy([[1,2],[1,2]])

import numpy as np
import pytest
from motion_toolbox.xy_smoothing import smooth_xy, significant_reversals


def test_vertical_motion_keeps_xy_constant():
    p = np.column_stack((np.ones(50), np.zeros(50), np.arange(50)))
    r = smooth_xy(p, .25)
    np.testing.assert_allclose(r['curve'], p[:, :2])
    assert r['converged']


def test_large_reversals_survive_and_ripples_are_removed():
    t = np.linspace(0, 1, 240)
    x = np.interp(t, [0, .25, .5, .75, 1], [0, 3, 0, 3, 0])
    p = np.column_stack((x, .04*np.sin(t*160)))
    r = smooth_xy(p, .2)
    assert r['curve'].shape == p.shape
    assert r['measured_max_deviation'] <= .2+1e-12
    assert len(significant_reversals(r['curve'][:, 0], 1.)) == 3
    assert np.ptp(r['curve'][:, 1]) < np.ptp(p[:, 1])
    assert r['curvature_energy'] < np.sum(np.diff(p, n=2, axis=0)**2)*.2


def test_rotation_translation_and_scale_equivariance():
    t = np.linspace(0, 3, 60)
    p = np.column_stack((t, .08*np.sin(t*15)))
    rotation = np.array([[.6, -.8], [.8, .6]])
    a = smooth_xy(p, .15)
    b = smooth_xy(p@rotation*1000+[500, -250], 150)
    np.testing.assert_allclose(b['curve'], a['curve']@rotation*1000+[500, -250], atol=1e-7)


def test_iteration_limit_does_not_claim_convergence():
    r = smooth_xy([[0, 0], [3, 1], [2, -1], [0, 0]], .5, max_iterations=1)
    assert not r['converged']
    assert r['measured_max_deviation'] <= .5+1e-12


@pytest.mark.parametrize('budget', [0, -1, float('nan'), float('inf')])
def test_invalid_budget(budget):
    with pytest.raises(ValueError):
        smooth_xy([[0, 0]], budget)

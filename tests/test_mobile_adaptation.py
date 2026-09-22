import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.mobile_adaptation import repair_weights, repair_anchors


def test_offset_ramps_preserve_unchanged_boundaries_and_plateau():
    w, lo, hi = repair_weights(101, 40, 60, 30)
    assert (lo, hi) == (10, 90)
    np.testing.assert_array_equal(w[:lo+1], 0.)
    np.testing.assert_array_equal(w[hi:], 0.)
    np.testing.assert_array_equal(w[40:61], 1.)
    np.testing.assert_allclose(w, w[::-1])
    assert 0 < w[lo+1] < .001
    assert 0 <= w.min() <= w.max() <= 1


def test_anchors_keep_height_extremes_and_reversal():
    targets = [Plane((abs(i-15)*.02, 0, 1+(i==8)), (0,0,1), (1,0,0)) for i in range(31)]
    assert {0,8,15,30} <= set(repair_anchors(targets,0,30))


def test_expanded_repairs_can_anticipate_before_first_failure():
    narrow,_,_=repair_weights(101,20,20,16)
    expanded,_,_=repair_weights(101,20,20,64)
    assert narrow[0]==0 and expanded[0]>0
    assert expanded[20]==1
    assert np.max(abs(np.diff(expanded)))<np.max(abs(np.diff(narrow)))


@pytest.mark.parametrize('yaw', [0., 30., -30.])
def test_adaptive_workflow_repairs_collision_and_validates_all_originals(monkeypatch, yaw):
    from motion_toolbox import mobile_base_workflow as workflow
    from test_mobile_base_workflow import Solver, World
    targets = [Plane((i*.02,0,1),(0,0,1),(1,0,0)) for i in range(33)]
    bases = [Plane((i*.02,-1,0),(0,1,0),(-1,0,0)) for i in range(33)]
    monkeypatch.setattr(workflow,'generate_base_path',lambda *a,**k:dict(
        base_planes=bases,target_indices=list(range(33)),smoothing={},centerline={}))
    world = World()
    world.last_failure = 'tool collision: tool[0] / wall'
    angle = np.deg2rad(yaw)
    expected_x = np.array([-np.sin(angle), np.cos(angle), 0.])
    def collision_check(q, b, **kw):
        np.testing.assert_allclose(b.xaxis, expected_x, atol=1e-12)
        np.testing.assert_allclose(b.zaxis, [0,0,1], atol=1e-12)
        return abs(q[0]-.32)>.005 or b.origin[1]<=-1.15
    world.is_valid = collision_check
    world.edge_is_valid = lambda *a,**k: (_ for _ in ()).throw(AssertionError('Swept check requested'))
    kw = dict(solver=Solver(),world=world,joint_ranges=[[-3,3]]*6,periodic=[False]*6,
              base_yaw_degrees=yaw)
    fixed = workflow.plan_base_path(targets,adapt_offsets=False,**kw)
    assert not fixed['fabrication_validated']
    assert fixed['unreachable_points']==[16]
    result = workflow.plan_base_path(targets,**kw)
    assert result['fabrication_validated'],result['status']
    assert len(result['base_planes'])==len(result['configurations'])==33
    assert result['repair_attempts']
    assert result['base_planes'][16].origin[1]<=-1.15
    for i in (0, -1):
        np.testing.assert_allclose(result['base_planes'][i].origin,bases[i].origin)
        np.testing.assert_allclose(result['base_planes'][i].xaxis,expected_x,atol=1e-12)
    assert result['base_yaw_degrees'] == yaw
    assert result['optimality_certified']
    assert not result['check_edges']

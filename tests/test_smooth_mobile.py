import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.smooth_mobile import moving_average, smooth_offset_proposals, plan_smooth_mobile


def wall(x, y=0, z=1):
    # +Z points towards +Y; footprint +X must face +Y.
    return Plane((x,y,z), (1,0,0), (0,0,-1))


def test_average_even_odd_and_long_windows():
    values = np.arange(10, dtype=float).reshape(-1,1)
    for w in (1,2,5,20):
        p = np.pad(values[:,0], ((w-1)//2,w-1-(w-1)//2), mode='edge')
        assert np.allclose(moving_average(values,w)[:,0], np.convolve(p,np.ones(w)/w,mode='valid'))


def test_suppresses_jitter_without_erasing_large_xy_features():
    xy = np.r_[np.linspace(0,3,201), np.linspace(3,0,201)]
    targets = [wall(x,.015*np.sin(i*1.7),z=1+.5*np.sin(i)) for i,x in enumerate(xy)]
    proposals, _ = smooth_offset_proposals(targets,Plane.world_xy(),wall_distances=[1],lateral_offsets=[0])
    _, bases, info = proposals[0]
    assert len(bases) == len(targets)
    positions = np.array([b.origin for b in bases])
    assert positions[:,0].max() > 2.5
    assert np.ptp(positions[:,1]) < .02
    assert np.allclose(positions[:,2],0)
    assert all(np.allclose(b.xaxis,(0,1,0)) for b in bases)
    assert info['max_arm_xy_reach'] <= 1.75


def test_curve_uses_local_normals_and_calibrated_mount():
    angles = np.linspace(-.6,.6,101)
    targets = [Plane((3*np.cos(a),3*np.sin(a),1),
                     (-np.sin(a),np.cos(a),0),(0,0,1)) for a in angles]
    mount = Plane((.3,.2,.5),(1,0,0),(0,1,0))
    proposals,_ = smooth_offset_proposals(targets,mount,windows=[1],wall_distances=[1],lateral_offsets=[0])
    for target,base in zip(targets,proposals[0][1]):
        arm = base.origin + .3*base.xaxis + .2*base.yaxis
        assert np.linalg.norm(base.origin[:2]) == pytest.approx(2)
        assert np.dot(base.xaxis[:2],target.origin[:2]/3) == pytest.approx(1)
        assert np.linalg.norm(target.origin[:2]-arm[:2]) == pytest.approx(np.hypot(.7,.2))


def test_default_sideways_offset_is_robot_y_and_wall_distance_is_separate():
    target = wall(0)
    proposals,_ = smooth_offset_proposals([target],Plane.world_xy(),windows=[1])
    assert {round(p[2]['wall_distance_metres'],2) for p in proposals} == {.4,.6,.8,1.,1.2}
    assert {p[2]['lateral_metres'] for p in proposals} == {-1.,1.}
    for _,bases,meta in proposals:
        base = bases[0]
        delta = target.origin-base.origin
        assert delta @ base.xaxis == pytest.approx(meta['wall_distance_metres'])
        assert abs(delta @ base.yaxis) == pytest.approx(1.)


def test_offset_repair_and_full_resolution_checks():
    import json
    targets = [wall(i*.01) for i in range(31)]
    checked, edges = set(), []
    def collision(q,b):
        if b.origin[1] < -.9:
            return False
        checked.add(round(q[0],6))
        return True
    result = plan_smooth_mobile(targets,Plane.world_xy(),windows=[10],wall_distances=[1,.8],lateral_offsets=[0],
        ik_solver=lambda t,b:[[float(t.origin[0])]], collision=collision,
        transition_check=lambda q0,b0,q1,b1: edges.append((q0,q1)) or True)
    assert len(result.base_planes) == len(result.configurations) == 31
    assert len(checked) == 31
    assert len(edges) == 30
    assert result.diagnostics[-1]['selected']['wall_distance_metres'] == .8
    assert len(result.diagnostics[-1]['attempts']) == 2
    json.dumps(result.diagnostics)  # Recording/GH diagnostics must not contain cycles.


def test_failure_budget_no_dense_fallback_or_relaxed_constraints():
    result = plan_smooth_mobile([wall(i*.01) for i in range(20)],Plane.world_xy(),
        ik_solver=lambda t,b:[[0]], transition_check=lambda *a:False, max_attempts=2)
    assert not result.configurations
    assert result.diagnostics[-1]['reason'] == 'smooth_proposals_exhausted'
    assert len(result.diagnostics[-1]['attempts']) == 2
    assert result.diagnostics[0]['reason'] == 'transition_blocked'


def test_smoothing_does_not_ignore_large_height_for_ik():
    result = plan_smooth_mobile([wall(0),wall(.1,z=10)],Plane.world_xy(),
        ik_solver=lambda t,b:[[0]] if t.origin[2] < 2 else [], max_attempts=1)
    assert not result.configurations
    assert result.candidate_counts[1] == 0


@pytest.mark.parametrize('options', [{'windows':[0]}, {'lateral_distance':-1}, {'wall_distances':[-1]}, {'lateral_offsets':[]}])
def test_invalid_settings(options):
    with pytest.raises(ValueError):
        smooth_offset_proposals([wall(0)],Plane.world_xy(),**options)

import json
import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.mobile_sections import plan_mobile_sections, blend_sections, _slice_options
from motion_toolbox.smooth_mobile import plan_smooth_mobile


def wall(i):
    return Plane((i*.01,0,1),(1,0,0),(0,0,-1))


def proposals(n):
    direction = np.tile([0.,1.],(n,1))
    tangent = np.tile([-1.,0.],(n,1))
    return [(0.,(np.column_stack((np.arange(n)*.01,np.full(n,y))),direction,tangent),
             dict(window=10,wall_distance_metres=-y,lateral_metres=0)) for y in (-1.,-.6)]


def section_ik(target,base):
    # A works only at the beginning, B only at the end. Both and their blend
    # work throughout the overlap; no single whole-path proposal works.
    i = round(target.origin[0]*100)
    if i < 20 and base.origin[1] > -.95:
        return []
    if i >= 40 and base.origin[1] < -.65:
        return []
    return [[i*.001]]


def test_connects_partial_paths_and_validates_all_original_targets():
    targets = [wall(i) for i in range(60)]
    edges = []
    exact_edges = set()
    def transition(q0,b0,q1,b1):
        key = (tuple(q0),tuple(b0.matrix.ravel()),tuple(q1),tuple(b1.matrix.ravel()))
        assert key not in exact_edges
        exact_edges.add(key)
        edges.append((q0[0],q1[0]))
        return True
    result = plan_mobile_sections(targets,proposals(60),Plane.world_xy(),
        section_size=40,ik_solver=section_ik,
        transition_check=transition)
    assert len(result.base_planes) == len(result.configurations) == 60
    assert result.base_planes[0].origin[1] == -1
    assert result.base_planes[-1].origin[1] == -.6
    assert len(result.diagnostics[-1]['joins']) == 2
    assert result.diagnostics[-1]['transition_cache_hits'] > 0
    assert set((round(a,3),round(b,3)) for a,b in edges) >= {
        (round(i*.001,3),round((i+1)*.001,3)) for i in range(59)}
    json.dumps(result.diagnostics)


def test_smooth_planner_enables_section_fallback():
    result = plan_smooth_mobile([wall(i) for i in range(60)],Plane.world_xy(),
        windows=[1],wall_distances=[1,.6],lateral_offsets=[0],
        max_attempts=2,repair_attempts=0,section_size=40,ik_solver=section_ik)
    assert len(result.base_planes) == 60
    assert result.diagnostics[-1]['selected']['strategy'] == 'connected_sections'
    json.dumps(result.diagnostics)


@pytest.mark.parametrize('kind',['collision','joint_step','placement'])
def test_rejects_invalid_joins_instead_of_concatenating_sections(kind):
    options = dict(ik_solver=section_ik)
    if kind == 'collision':
        options['transition_check'] = lambda q0,b0,q1,b1: abs(b1.origin[1]-b0.origin[1]) < 1e-10
    elif kind == 'joint_step':
        options['ik_solver'] = lambda t,b: ([[0 if b.origin[1] < -.8 else 3]] if section_ik(t,b) else [])
        options['max_joint_step'] = .1
    else:
        options['base_valid'] = lambda t,b: b.origin[1] < -.99 or b.origin[1] > -.61
    result = plan_mobile_sections([wall(i) for i in range(60)],proposals(60),Plane.world_xy(),
        section_size=40,**options)
    assert not result.base_planes and not result.configurations
    assert result.diagnostics[-1]['reason'] == 'section_connection_failed'
    assert result.diagnostics[-1]['validated_prefix_targets'] == 40


def test_overlap_uses_short_yaw_arc_and_smooth_endpoints():
    def p(yaw):
        return Plane((0,0,0),(np.cos(yaw),np.sin(yaw),0),(-np.sin(yaw),np.cos(yaw),0))
    joined = blend_sections([p(np.deg2rad(179))]*10,[p(np.deg2rad(-179))]*10,5)
    assert len(joined) == 15
    assert all(b.xaxis[0] < -.99 for b in joined)
    assert np.allclose(joined[5].xaxis,p(np.deg2rad(179)).xaxis)
    assert np.allclose(joined[9].xaxis,p(np.deg2rad(-179)).xaxis)


@pytest.mark.parametrize('has_start',[False,True])
def test_section_timing_matches_original_edges(has_start):
    opts = dict(time_intervals=list(range(1,11)),start_base=Plane.world_xy() if has_start else None,
                current_pose=[0] if has_start else None)
    local = _slice_options(opts,3,7)
    assert local['start_base'] is None and local['current_pose'] is None
    assert local['time_intervals'] == opts['time_intervals'][3+int(has_start):6+int(has_start)]
    assert _slice_options(opts,0,7)['time_intervals'] == opts['time_intervals'][:6+int(has_start)]

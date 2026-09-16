import math
from functools import partial
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.base_planning import plan_mobile_base
from motion_toolbox.mobile_planning import plan_mobile_sparse


def plane(x=0, yaw=0):
    return Plane((x, 0, 0), (math.cos(yaw), math.sin(yaw), 0), (-math.sin(yaw), math.cos(yaw), 0))


@pytest.mark.parametrize('options,bases,solver,reason', [
    ({'max_base_step': .1}, [plane(), plane(.2)], lambda t,b: [[0]], 'base_step'),
    ({'max_yaw_step': .1}, [plane(), plane(yaw=.2)], lambda t,b: [[0]], 'yaw_step'),
    ({'max_joint_step': .1}, [plane(), plane()], lambda t,b: [[t.origin[0]]], 'joint_step'),
    ({'max_base_speed': .1}, [plane(), plane(.2)], lambda t,b: [[0]], 'base_speed'),
    ({'max_yaw_speed': .1}, [plane(), plane(yaw=.2)], lambda t,b: [[0]], 'yaw_speed'),
    ({'max_joint_speed': .1}, [plane(), plane()], lambda t,b: [[t.origin[0]]], 'joint_speed'),
    ({'transition_check': lambda *a: False}, [plane(), plane()], lambda t,b: [[0]], 'transition_check'),
])
def test_reports_constraint_including_graph_prefilters(options, bases, solver, reason):
    result = plan_mobile_base([plane(), plane(.2)], [[b] for b in bases],
        ik_solver=solver, time_intervals=[1], **options)
    assert not result.configurations
    detail = result.diagnostics[0]
    assert (detail['from_target'], detail['to_target']) == (0, 1)
    assert detail['rejection_counts'] == {reason: 1}
    if reason != 'transition_check':
        assert detail['constraint_examples'][reason]['limit'] == .1


def test_collision_pair_and_start_transition():
    class Scene:
        last_failure = 'tool / wall'
        def check(self, *args):
            return False
    result = plan_mobile_base([plane()], [[plane()]], ik_solver=lambda t,b:[[0]],
        start_base=plane(), current_pose=[0], transition_check=partial(Scene().check))
    detail = result.diagnostics[0]
    assert detail['from_target'] == 'start'
    assert detail['constraint_examples']['transition_collision']['detail'] == 'tool / wall'


def test_connected_selection_searches_past_disconnected_feasible_bases():
    result = plan_mobile_base([plane(),plane(.1)],
        [[plane()], [plane(5),plane(6),plane(.1)]],
        ik_solver=lambda t,b:[[0]], max_feasible_bases=1, _connected_candidates=True,
        base_valid=lambda t,b: t.origin[0] == 0 or b.origin[0] > 0)
    assert len(result.base_planes) == 2
    assert result.base_planes[1].origin[0] == pytest.approx(.1)


def test_local_expansion_recovers_a_discarded_predecessor():
    targets = [plane(), plane(.1)]
    def valid(t,b):
        return b.origin[0] in ([0., 1.] if t.origin[0] == 0 else [1.])
    result = plan_mobile_sparse(targets, [[plane(),plane(1)], [plane(1)]],
        ik_solver=lambda t,b:[[0]], base_valid=valid, max_feasible_bases=1)
    assert len(result.base_planes) == 2
    assert all(b.origin[0] == 1 for b in result.base_planes)
    assert any('local_repairs' in d for d in result.diagnostics)


def test_blocked_fallback_stops_before_later_regions_and_keeps_unknown_counts():
    requested = []
    def layer(i,t):
        requested.append(i)
        return [plane()]
    result = plan_mobile_sparse([plane(),plane(.1),plane(.2)], layer,
        max_gap=1, max_feasible_bases=1, ik_solver=lambda t,b:[[0]],
        transition_check=lambda *a: False)
    assert requested == [0,1]
    assert result.candidate_counts[-1] is None
    assert result.diagnostics[0]['reason'] == 'transition_blocked'


def test_periodic_joint_wrap_is_preserved():
    result = plan_mobile_base([plane(),plane(.1)], [[plane()],[plane()]],
        ik_solver=lambda t,b:[[3.1 if t.origin[0] == 0 else -3.1]],
        periodic=[True], max_joint_step=.2, _connected_candidates=True)
    assert len(result.configurations) == 2

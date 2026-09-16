from contextlib import contextmanager
import pytest
from motion_toolbox import runtime
from motion_toolbox.geometry import Plane
from motion_toolbox.base_planning import plan_mobile_base
from motion_toolbox.graph import shortest_path


def clock(monkeypatch):
    now = [0.]
    monkeypatch.setattr(runtime,'monotonic',lambda: now[0])
    return now


def test_deadline_is_hardcoded_45_minutes_and_does_not_leak(monkeypatch):
    assert runtime.COMPONENT_RUNTIME_SECONDS == 2700
    now = clock(monkeypatch)
    @runtime.component_deadline
    def work(expire):
        now[0] += 2700 if expire else 1
        runtime.check_deadline('test stage')
        return 7
    with pytest.raises(runtime.PlanningTimeout,match='45-minute') as caught:
        work(True)
    assert caught.value.stage == 'test stage'
    assert caught.value.elapsed_seconds == 2700
    assert runtime._deadline.get() is None
    assert work(False) == 7


def test_nested_calls_do_not_restart_budget_and_cleanup_runs(monkeypatch):
    now = clock(monkeypatch)
    closed = []
    @contextmanager
    def resource():
        try:
            yield
        finally:
            closed.append(True)
    @runtime.component_deadline
    def child():
        now[0] = 2701
        runtime.check_deadline('child')
    @runtime.component_deadline
    def parent():
        with resource():
            now[0] = 2690
            child()
    with pytest.raises(runtime.PlanningTimeout):
        parent()
    assert closed == [True]
    assert runtime._deadline.get() is None


def test_ik_loop_exits_before_remaining_targets(monkeypatch):
    now = clock(monkeypatch)
    calls = []
    def ik(t,b):
        calls.append(t)
        now[0] = 2701
        return [[0]]
    @runtime.component_deadline
    def work():
        return plan_mobile_base([Plane.world_xy()]*100,[[Plane.world_xy()]]*100,ik_solver=ik)
    with pytest.raises(runtime.PlanningTimeout):
        work()
    assert len(calls) == 1


def test_graph_checks_inside_layer_search(monkeypatch):
    now = clock(monkeypatch)
    calls = []
    def edge(*args):
        calls.append(args)
        now[0] = 2701
        return True
    @runtime.component_deadline
    def work():
        return shortest_path([[[i*.01] for i in range(100)]]*3,edge_valid=edge,count_paths=False)
    with pytest.raises(runtime.PlanningTimeout):
        work()
    assert len(calls) == 1


def test_overdue_last_operation_cannot_return_success(monkeypatch):
    now = clock(monkeypatch)
    @runtime.component_deadline
    def work():
        now[0] = 2800
        return 'success'
    with pytest.raises(runtime.PlanningTimeout):
        work()


def test_no_deadline_does_not_change_numeric_api():
    runtime.check_deadline('outside component')
    assert shortest_path([[[0]],[[.1]]]).configurations == [[0.],[.1]]


@pytest.mark.parametrize('script',['grasshopper.py','grasshopper_mobile_base.py'])
def test_component_returns_timeout_status_and_empty_outputs(monkeypatch,script):
    import runpy
    from pathlib import Path
    times = iter([0.,2700.])
    monkeypatch.setattr(runtime,'monotonic',lambda:next(times,2700.))
    result = runpy.run_path(str(Path(__file__).resolve().parents[1]/'examples'/script))
    assert '45-minute runtime limit' in result['status']
    assert result['timings']['timed_out'] is True
    assert result['configurations'] == []
    assert result['base_result'] == []
    assert result['diagnostics']

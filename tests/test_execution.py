import pytest
from motion_toolbox import execution


class Controller:
    def __init__(self):
        self.states = []
    def read(self):
        return (1, 2, 2)
    def write(self, state):
        self.states.append(state)


def test_nested_performance_requests_restore_exact_thread_policy(monkeypatch):
    controller = Controller()
    monkeypatch.setattr(execution, '_controller', lambda: controller)
    with execution.performance_scope() as outer:
        with execution.performance_scope() as inner:
            assert inner is outer
            assert inner['applied']
            assert controller.states == [(1, 3, 2)]
    assert controller.states == [(1, 3, 2), (1, 2, 2)]
    assert outer['restored']


def test_interruption_restores_policy(monkeypatch):
    controller = Controller()
    monkeypatch.setattr(execution, '_controller', lambda: controller)
    with pytest.raises(KeyboardInterrupt):
        with execution.performance_scope():
            raise KeyboardInterrupt()
    assert controller.states[-1] == (1, 2, 2)
    assert execution._local.depth == 0


def test_unavailable_policy_preserves_computation(monkeypatch):
    def unavailable():
        raise OSError('unsupported policy API')
    monkeypatch.setattr(execution, '_controller', unavailable)
    @execution.high_qos
    def compute():
        return dict(value=42)
    result = compute()
    assert result['value'] == 42
    assert not result['execution_policy']['applied']
    assert 'unsupported' in result['execution_policy']['error']


def test_restoration_failure_does_not_replace_result(monkeypatch):
    controller = Controller()
    def write(state):
        controller.states.append(state)
        if len(controller.states) == 2:
            raise OSError('restore failed')
    controller.write = write
    monkeypatch.setattr(execution, '_controller', lambda: controller)
    @execution.high_qos
    def compute():
        return dict(value=42)
    with pytest.warns(RuntimeWarning, match='restore'):
        result = compute()
    assert result['value'] == 42
    assert result['execution_policy']['restored'] is False


def test_default_platform_policy_is_reported(monkeypatch):
    monkeypatch.setattr(execution, '_controller', lambda: None)
    with execution.performance_scope() as policy:
        assert not policy['applied']
    assert policy['restored'] is None


def test_decorator_reports_restoration_and_preserves_signature(monkeypatch):
    import inspect
    controller = Controller()
    monkeypatch.setattr(execution, '_controller', lambda: controller)
    @execution.high_qos
    def compute(value=42):
        return dict(value=value)
    assert str(inspect.signature(compute)) == '(value=42)'
    assert compute()['execution_policy']['restored']


def test_cancellation_exception_survives_rhino_reload():
    import importlib
    from motion_toolbox.execution import PlanningCancelled
    importlib.reload(execution)
    with pytest.raises(PlanningCancelled):
        execution.check_cancel(lambda: True)

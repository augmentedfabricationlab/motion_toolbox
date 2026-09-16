import threading
import time
import runpy
from pathlib import Path
import pytest
from motion_toolbox import mobile_component_runtime as runtime


def test_success_and_exception_propagate():
    assert runtime.MOBILE_COMPONENT_TIMEOUT_SECONDS == 2700
    assert runtime.run_mobile_component(lambda: 42) == 42
    def fail():
        raise ValueError('original failure')
    with pytest.raises(ValueError,match='original failure'):
        runtime.run_mobile_component(fail)
    assert not runtime._tasks


def test_timeout_returns_without_waiting_for_worker_and_prevents_overlap(monkeypatch):
    monkeypatch.setattr(runtime,'MOBILE_COMPONENT_TIMEOUT_SECONDS',.03)
    release = threading.Event()
    finished = threading.Event()
    def work():
        try:
            release.wait(3)
            return 'late result'
        finally:
            finished.set()
    started = time.monotonic()
    try:
        with pytest.raises(runtime.MobileComponentTimeout):
            runtime.run_mobile_component(work,component_key='test')
        assert time.monotonic()-started < 1
        assert not finished.is_set()
        with pytest.raises(RuntimeError,match='Previous mobile-base worker'):
            runtime.run_mobile_component(lambda: None,component_key='test')
    finally:
        release.set()
        assert finished.wait(2)
        # Worker cleanup removes the guard after the user function's finally.
        limit = time.monotonic()+2
        while 'test' in runtime._tasks and time.monotonic()<limit:
            time.sleep(.005)
    assert runtime.run_mobile_component(lambda: 'new run',component_key='test') == 'new run'


def test_cancel_interrupts_toolbox_work_and_runs_cleanup(monkeypatch):
    monkeypatch.setattr(runtime,'MOBILE_COMPONENT_TIMEOUT_SECONDS',.03)
    finished = threading.Event()
    from motion_toolbox.geometry import Plane
    def work():
        try:
            while True:
                Plane.world_xy()
        finally:
            finished.set()
    with pytest.raises(runtime.MobileComponentTimeout):
        runtime.run_mobile_component(work,component_key='cancel')
    assert finished.wait(2)


def test_only_mobile_script_uses_timeout_wrapper(monkeypatch):
    calls = []
    def expired(*args,**kwargs):
        calls.append(True)
        raise runtime.MobileComponentTimeout(2700)
    monkeypatch.setattr(runtime,'run_mobile_component',expired)
    folder = Path(__file__).resolve().parents[1]/'examples'
    out = runpy.run_path(str(folder/'grasshopper_mobile_base.py'))
    assert calls == [True]
    assert out['timings']['timed_out']
    assert out['base_planes'] == out['configurations'] == []
    assert '45-minute' in out['status']
    runpy.run_path(str(folder/'grasshopper.py'))
    assert calls == [True]

"""Time-bound only the Grasshopper mobile-base component's worker job."""
from contextvars import copy_context
from pathlib import Path
import sys
import threading
from time import monotonic

MOBILE_COMPONENT_TIMEOUT_SECONDS = 45 * 60
_tasks = {}
_lock = threading.Lock()
_package = str(Path(__file__).resolve().parent)


class MobileComponentTimeout(TimeoutError):
    timed_out = True

    def __init__(self, elapsed):
        self.elapsed_seconds = elapsed
        super().__init__('Mobile base planning stopped waiting after the 45-minute limit. '
                         'Worker cancellation requested; no path returned.')


class _Cancelled(Exception):
    pass


def run_mobile_component(function, *args, component_key='mobile_base', **kwargs):
    """Return to GH on expiry without joining a worker that is still unwinding.

    Never kill a Python/native thread or disconnect a scene it is still using.
    Cancellation is raised at the next toolbox function call; native operations
    must return first. A timed-out worker blocks another run of this component
    until cleanup finishes. The worker never writes Grasshopper output values.
    """
    started = monotonic()
    task = dict(done=threading.Event(), cancel=threading.Event(), result=None, error=None)
    with _lock:
        if component_key in _tasks:
            raise RuntimeError('Previous mobile-base worker is still running or cleaning up. '
                               'Recompute after it finishes; a duplicate job was not started.')
        _tasks[component_key] = task
    context = copy_context()

    def trace(frame, event, arg):
        # No per-line tracing overhead. Heavy planner loops repeatedly call
        # toolbox helpers. Raising once disables tracing and lets finally/with
        # cleanup proceed normally. Never interrupt recording or close methods.
        if (event == 'call' and task['cancel'].is_set()
                and frame.f_code.co_filename.startswith(_package)
                and not frame.f_code.co_filename.endswith(('recording.py','mobile_component_runtime.py'))
                and frame.f_code.co_name not in ('close','__exit__','__del__')):
            raise _Cancelled('Mobile component deadline exceeded')
        return None

    def execute():
        old_trace = sys.gettrace()
        try:
            sys.settrace(trace)
            task['result'] = context.run(function, *args, **kwargs)
        except BaseException as error:
            task['error'] = error
        finally:
            sys.settrace(old_trace)
            with _lock:
                _tasks.pop(component_key, None)
            task['done'].set()

    worker = threading.Thread(target=execute, name='Motion mobile base planner', daemon=True)
    try:
        worker.start()
    except BaseException:
        with _lock:
            _tasks.pop(component_key, None)
        raise
    remaining = max(0., MOBILE_COMPONENT_TIMEOUT_SECONDS-(monotonic()-started))
    if not task['done'].wait(remaining):
        task['cancel'].set()
        raise MobileComponentTimeout(monotonic()-started)
    if task['error'] is not None:
        raise task['error']
    if monotonic()-started >= MOBILE_COMPONENT_TIMEOUT_SECONDS:
        raise MobileComponentTimeout(monotonic()-started)
    return task['result']

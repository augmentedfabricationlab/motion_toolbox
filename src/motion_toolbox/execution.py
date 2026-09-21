"""Scoped CPU throughput requests for synchronous planning work.

Windows can lower the QoS of hidden/background applications. Request HighQoS
on the calling thread while computing, then restore its exact prior policy.
Priority, CPU affinity and the system power plan are unaffected.
"""
from contextlib import contextmanager
from functools import wraps
import sys
import threading
import warnings
from .recording import event

_local = threading.local()


class _WindowsThreadPolicy:
    def __init__(self):
        import ctypes
        from ctypes import wintypes
        class State(ctypes.Structure):
            _fields_ = [('Version', wintypes.DWORD), ('ControlMask', wintypes.DWORD),
                        ('StateMask', wintypes.DWORD)]
        self.ctypes, self.State = ctypes, State
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.GetCurrentThread.restype = wintypes.HANDLE
        self.handle = kernel.GetCurrentThread()
        self.get, self.set = kernel.GetThreadInformation, kernel.SetThreadInformation
        for function in (self.get, self.set):
            function.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
            function.restype = wintypes.BOOL

    def read(self):
        state = self.State(1, 0, 0)
        if not self.get(self.handle, 3, self.ctypes.byref(state), self.ctypes.sizeof(state)):
            raise self.ctypes.WinError(self.ctypes.get_last_error())
        return state.Version, state.ControlMask, state.StateMask

    def write(self, values):
        state = self.State(*values)
        if not self.set(self.handle, 3, self.ctypes.byref(state), self.ctypes.sizeof(state)):
            raise self.ctypes.WinError(self.ctypes.get_last_error())


def _controller():
    return _WindowsThreadPolicy() if sys.platform == 'win32' else None


@contextmanager
def performance_scope():
    """Advisory HighQoS for this thread, with nested scopes and error fallback."""
    local = _local  # Retain the same thread-local object across a GH module reload.
    if getattr(local, 'depth', 0):
        local.depth += 1
        try:
            yield local.policy
        finally:
            local.depth -= 1
        return
    policy = dict(request='high_qos', scope='calling_thread', thread_id=threading.get_native_id(),
                  applied=False, restored=None)
    controller, before, restore_needed = None, None, False
    local.depth, local.policy = 1, policy
    try:
        try:
            controller = _controller()
            if controller is None:
                policy['reason'] = 'Platform uses its default CPU policy'
            else:
                before = controller.read()
                requested = (before[0], before[1] | 1, before[2] & ~1)
                policy.update(before=list(before), requested_state=list(requested))
                restore_needed = True
                controller.write(requested)
                policy['applied'] = True
        except Exception as error:
            policy['error'] = str(error)
        event('execution.cpu_policy', **policy)
        yield policy
    finally:
        if restore_needed:
            try:
                controller.write(before)
                policy['restored'] = True
            except Exception as error:
                policy.update(restored=False, restoration_error=str(error))
                try:
                    warnings.warn('Could not restore planning thread CPU policy: '+str(error), RuntimeWarning)
                except Warning:
                    pass  # Warning filters must not replace a planner result/error.
        local.depth = 0
        del local.policy
        event('execution.cpu_policy_finished', **policy)


def high_qos(function):
    """Request throughput independently of logging; annotate mapping results."""
    @wraps(function)
    def wrapped(*args, **kwargs):
        with performance_scope() as policy:
            result = function(*args, **kwargs)
        if isinstance(result, dict):
            result['execution_policy'] = dict(policy)
        return result
    return wrapped

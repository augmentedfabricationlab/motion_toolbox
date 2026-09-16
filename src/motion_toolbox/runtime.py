"""Per-invocation planning deadline; never terminate Rhino's execution thread."""
from contextvars import ContextVar
from functools import wraps
from time import monotonic

COMPONENT_RUNTIME_SECONDS = 45 * 60
_deadline = ContextVar('motion_planning_deadline', default=None)


class PlanningTimeout(TimeoutError):
    timed_out = True

    def __init__(self, started, stage):
        self.elapsed_seconds = monotonic()-started
        self.stage = stage
        super().__init__('Planning stopped: 45-minute runtime limit reached during {}. '
                         'No complete validated path returned.'.format(stage))


def check_deadline(stage='planning'):
    deadline = _deadline.get()
    if deadline is not None and monotonic() >= deadline[1]:
        raise PlanningTimeout(deadline[0], stage)


def component_deadline(function):
    """Nested planning shares one deadline; finally always restores the context.

    Checks are cooperative between bounded Python/native operations. A native
    call stuck inside a driver cannot be preempted safely in the Rhino process.
    """
    @wraps(function)
    def wrapped(*args, **kwargs):
        started = monotonic()
        active = _deadline.get()
        limit = started+COMPONENT_RUNTIME_SECONDS
        deadline = active if active is not None and active[1] <= limit else (started,limit)
        token = _deadline.set(deadline)
        try:
            check_deadline('component start')
            result = function(*args, **kwargs)
            check_deadline('component completion')
            return result
        finally:
            _deadline.reset(token)
    return wrapped

"""Synthetic smooth-offset timing; not a real-robot collision benchmark."""
import json
from time import perf_counter
import numpy as np
from motion_toolbox.geometry import Plane
from motion_toolbox.smooth_mobile import plan_smooth_mobile
from motion_toolbox.recording import suspend_recording


def run():
    targets = [Plane((i*.002,.01*np.sin(i*.8),1+.1*np.sin(i*.2)),
                     (1,0,0),(0,0,-1)) for i in range(1607)]
    calls = dict(ik=0, collision=0, transition=0)
    def ik(t,b):
        calls['ik'] += 1
        return [[float(t.origin[0])*.1]]
    def collision(q,b):
        calls['collision'] += 1
        return True
    def edge(*args):
        calls['transition'] += 1
        return True
    started = perf_counter()
    with suspend_recording():
        result = plan_smooth_mobile(targets,Plane.world_xy(),ik_solver=ik,
                                   collision=collision,transition_check=edge)
    assert len(result.base_planes) == len(result.configurations) == len(targets)
    assert calls['collision'] == len(targets)
    assert calls['transition'] == len(targets)-1
    return dict(seconds=perf_counter()-started, targets=len(targets), calls=calls,
                selected=result.diagnostics[-1]['selected'])


if __name__ == '__main__':
    print(json.dumps(run(),indent=2))

"""Synthetic overlapping-section workload; callbacks are not a robot model."""
import json
from time import perf_counter
import numpy as np
from motion_toolbox.geometry import Plane
from motion_toolbox.mobile_sections import plan_mobile_sections
from motion_toolbox.recording import suspend_recording


def run(n=1607):
    targets = [Plane((i*.002,0,1),(1,0,0),(0,0,-1)) for i in range(n)]
    x = np.arange(n)*.002
    direction = np.tile([0.,1.],(n,1))
    tangent = np.tile([-1.,0.],(n,1))
    proposals = [(0.,(np.column_stack((x,np.full(n,y))),direction,tangent),{'wall_y':y})
                 for y in (-1.,-.6)]
    calls = dict(ik=0,collision=0,transition=0)
    def ik(t,b):
        calls['ik'] += 1
        i = round(t.origin[0]/.002)
        if (i<200 and b.origin[1]>-.95) or (i>=400 and b.origin[1]<-.65):
            return []
        return [[i*.0001]]
    def collision(q,b):
        calls['collision'] += 1
        return True
    def edge(*args):
        calls['transition'] += 1
        return True
    started = perf_counter()
    with suspend_recording():
        result = plan_mobile_sections(targets,proposals,Plane.world_xy(),
            ik_solver=ik,collision=collision,transition_check=edge)
    assert len(result.base_planes) == len(result.configurations) == n
    assert result.base_planes[0].origin[1] == -1.
    assert result.base_planes[-1].origin[1] == -.6
    return dict(seconds=perf_counter()-started,targets=n,
                sections=len(result.diagnostics[-1]['joins']),calls=calls)


if __name__ == '__main__':
    print(json.dumps(run(),indent=2))

"""Synthetic deterministic dense/sparse mobile search comparison; run from repo."""
import json
from time import perf_counter
from motion_toolbox.geometry import Plane
from motion_toolbox.base_planning import plan_mobile_base
from motion_toolbox.mobile_planning import plan_mobile_sparse
from motion_toolbox.recording import suspend_recording


def run():
    targets = [Plane((i*.005,0,0),(1,0,0),(0,1,0)) for i in range(201)]
    layers = [[Plane((t.origin[0],j*.02,0),(1,0,0),(0,1,0)) for j in range(12)] for t in targets]
    report = {}
    for name, planner in [('dense',plan_mobile_base),('sparse',plan_mobile_sparse)]:
        calls = [0]
        def ik(t,b):
            calls[0] += 1
            return [[float(t.origin[0]-b.origin[0])]]
        started = perf_counter()
        with suspend_recording():
            result = planner(targets,layers,ik_solver=ik)
        report[name] = dict(seconds=perf_counter()-started,ik_calls=calls[0],
                            targets=len(result.base_planes),cost=result.cost)
    report['speedup'] = report['dense']['seconds']/report['sparse']['seconds']
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    run()

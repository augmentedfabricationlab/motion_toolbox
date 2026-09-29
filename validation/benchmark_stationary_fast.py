"""Reproducible stationary benchmark: synthetic IK layers, real Bullet collisions.

Run with python -m validation.benchmark_stationary_fast. No captured robot data
is required. Timings include scene setup and use medians of three runs per mode.
"""
import argparse
import json
import os
import platform
from pathlib import Path
from statistics import median
from tempfile import TemporaryDirectory
from time import perf_counter

import numpy as np
from motion_toolbox.base_planning import find_stationary_base
from motion_toolbox.collision import PybulletServer
from motion_toolbox.geometry import Plane
from motion_toolbox.stationary_region import StationaryRegion
from motion_toolbox import __version__

URDF = '''<robot name="benchmark">
<link name="base"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size=".2 .2 .2"/></geometry></collision></link>
<link name="arm"><inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial><collision><origin xyz=".6 0 0"/><geometry><box size=".4 .1 .1"/></geometry></collision></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
<axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint></robot>'''


def benchmark():
    os.environ['TOOLBOX_RECORDING'] = '0'
    targets = [Plane((i*.001, 0, 1), (1, 0, 0), (0, 0, -1)) for i in range(150)]
    base = Plane((0, -1, 0), (1, 0, 0), (0, 1, 0))
    region = StationaryRegion(targets, Plane.world_xy(), projected=True)
    rows = [[float(q)] for q in np.linspace(0, 2.5, 48)]
    report = dict(package_version=__version__, python=platform.python_version(),
        platform=platform.system(), repeats=3, statistic='median',
        workload='Synthetic IK candidates; real PyBullet configuration collisions',
        targets=len(targets), candidates_per_target=len(rows), cases={})
    with TemporaryDirectory() as folder:
        path = Path(folder)/'robot.urdf'
        path.write_text(URDF)
        for heavy in (False, True):
            runs = {False: [], True: []}
            results = {}
            for repeat in range(3):
                for fast in ((False, True) if repeat % 2 == 0 else (True, False)):
                    started = perf_counter()
                    with PybulletServer(path, check_static_self_collisions=False) as world:
                        if heavy:
                            world.add_box([.35, .35, .1], plane=Plane((.6, -.8, 0), (1, 0, 0), (0, 1, 0)))
                        setup = perf_counter()-started
                        checks = []
                        def check(q, b):
                            checks.append(1)
                            return world.is_valid(q, b)
                        result = find_stationary_base(targets, [base], ik_solver=lambda t,b:rows,
                            collision=check, base_collision=world.is_base_valid,
                            objective='heuristic', placement_region=region,
                            fast_validation=fast, count_paths=False)
                        timings = result.diagnostics[-1]['timings']
                        runs[fast].append(dict(setup_seconds=setup, total_seconds=perf_counter()-started,
                            collision_checks=len(checks), **timings))
                        results[fast] = result
            assert results[False].configurations == results[True].configurations
            assert results[False].cost == results[True].cost
            assert results[True].configurations
            report['cases']['collision_heavy' if heavy else 'clear'] = {
                'fast' if fast else 'exhaustive': {key: median(r[key] for r in samples)
                    for key in samples[0]} for fast, samples in runs.items()}
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = json.dumps(benchmark(), indent=2)
    if args.output:
        args.output.write_text(report+'\n')
    print(report)

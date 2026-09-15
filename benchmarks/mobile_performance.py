"""Compare current code to a Git revision with unchanged synthetic inputs.

Run: python benchmarks/mobile_performance.py --baseline 4ef10ee
Outputs timings and exact parity checks; generated logs stay in temporary dirs.
These workloads measure graph checks, region generation and recording overhead,
not the user's complete robot scene.
"""
import argparse
from contextlib import closing
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
from time import perf_counter
import types
import numpy as np
from motion_toolbox import graph, recording, stationary_region
from motion_toolbox.geometry import Plane
from motion_toolbox.collision import PybulletServer


def previous(name, revision):
    source = subprocess.check_output(['git','show',revision+':src/motion_toolbox/'+name+'.py'],text=True)
    module = types.ModuleType('motion_toolbox._baseline_'+name)
    module.__package__ = 'motion_toolbox'
    sys.modules[module.__name__] = module
    exec(compile(source, '<baseline_'+name+'>', 'exec'), module.__dict__)
    return module


def run(revision):
    old = {n:previous(n,revision) for n in ('graph','recording','stationary_region','collision')}
    report = {'baseline':revision}
    layers = [np.random.default_rng(i).uniform(-.3,.3,(96,6)).tolist() for i in range(5)]
    graph_results = []
    for label,module in [('before',old['graph']),('after',graph)]:
        calls = [0]
        def edge(i,a,b):
            calls[0] += 1
            # Deterministic stand-in for an expensive collision/constraint callback.
            for _ in range(8):
                np.sin(np.arange(64)+a+b).sum()
            return (a+b+i)%4 == 0
        start = perf_counter()
        with recording.suspend_recording():
            result = module.shortest_path(layers,count_paths=False,edge_valid=edge)
        report['graph_'+label] = dict(seconds=perf_counter()-start,edge_checks=calls[0])
        graph_results.append(result)
    assert graph_results[0].indices == graph_results[1].indices
    assert graph_results[0].cost == graph_results[1].cost
    report['graph_exact_parity'] = True
    with tempfile.TemporaryDirectory(prefix='mobile-bullet-') as directory:
        urdf = Path(directory)/'robot.urdf'
        urdf.write_text('''<robot name="benchmark"><link name="base"/>
          <link name="arm"><collision><origin xyz="0.6 0 0"/>
          <geometry><box size="0.4 0.1 0.1"/></geometry></collision></link>
          <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
          <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
          </joint></robot>''',encoding='utf-8')
        with recording.suspend_recording(), PybulletServer(urdf) as scene:
            scene.add_box([.08]*3,plane=Plane((.6,0,0),(1,0,0),(0,1,0)))
            base = Plane.world_xy()
            safe = [[float(q)] for q in np.linspace(-1,1,32) if scene.is_valid([q],base)]
            bullet_results = []
            for label,module in [('before',old['graph']),('after',graph)]:
                calls = [0]
                def edge(i,a,b):
                    calls[0] += 1
                    return scene.edge_is_valid(safe[a],base,safe[b],base)
                start = perf_counter()
                result = module.shortest_path([safe]*3,count_paths=False,edge_valid=edge)
                report['bullet_'+label] = dict(seconds=perf_counter()-start,edge_checks=calls[0])
                bullet_results.append(result)
            assert bullet_results[0].indices == bullet_results[1].indices
            assert bullet_results[0].cost == bullet_results[1].cost
            report['bullet_exact_parity'] = True
        parts = ''.join('''<link name="part{0}"><collision><geometry><box size="0.1 0.1 0.1"/>
          </geometry></collision></link><joint name="fixed{0}" type="fixed"><parent link="base"/>
          <child link="part{0}"/><origin xyz="{1} 0 0"/></joint>'''.format(i,(i-4)*.1) for i in range(9))
        urdf.write_text('<robot name="base"><link name="base"/>'+parts+'</robot>',encoding='utf-8')
        base_results = []
        with recording.suspend_recording(), PybulletServer(urdf,joint_names=[]) as scene:
            for i in range(42):
                scene.add_box([.05]*3,plane=Plane((5+i,0,0),(1,0,0),(0,1,0)))
            scene.add_box([.05]*3,plane=Plane.world_xy())
            bases = [Plane((x,0,0),(1,0,0),(0,1,0)) for x in np.linspace(-2,2,100)]
            query = scene.p.getClosestPoints
            for label,method in [('before',old['collision'].PybulletServer.is_base_valid),
                                  ('after',PybulletServer.is_base_valid)]:
                calls = [0]
                def counted(*args,**kwargs):
                    calls[0] += 1
                    return query(*args,**kwargs)
                scene.p.getClosestPoints = counted
                start = perf_counter()
                result = [(method(scene,b),scene.last_failure) for b in bases]
                report['base_collision_'+label] = dict(seconds=perf_counter()-start,queries=calls[0])
                base_results.append(result)
            assert base_results[0] == base_results[1]
            report['base_collision_exact_parity'] = True
    poses = []
    for label,module in [('before',old['stationary_region']),('after',stationary_region)]:
        start = perf_counter()
        output = []
        with recording.suspend_recording():
            for i in range(20):
                target = Plane((i*.03,0,.2),(1,0,0),(0,0,-1))
                region = module.StationaryRegion([target],Plane((.275,0,1),(1,0,0),(0,1,0)),projected=True)
                output.extend(b.matrix for b in region.candidates()[0])
        report['regions_'+label] = dict(seconds=perf_counter()-start,planes=len(output))
        poses.append(np.array(output))
    assert np.array_equal(*poses)
    report['region_exact_parity'] = True
    for label,module in [('before',old['recording']),('after',recording)]:
        with tempfile.TemporaryDirectory(prefix='mobile-perf-') as directory:
            with module.ResearchRun(directory) as run:
                def child():
                    for i in range(10):
                        run.metric('filter_'+str(i),i)
                    run.event('filter',accepted=8,rejected=2)
                    return {'candidate_count':8}
                def workload():
                    for _ in range(1000):
                        run.call(child,(),{})
                    return 1000
                run.source(child)
                run.source(workload)
                start = perf_counter()
                run.call(workload,(),{})
                seconds = perf_counter()-start
            with closing(sqlite3.connect(run.path/'run.sqlite3')) as db:
                counts = {t:db.execute('SELECT count(*) FROM '+t).fetchone()[0]
                          for t in ('steps','events','metrics')}
            report['recording_'+label] = dict(seconds=seconds,**counts)
    assert all(report['recording_before'][k] == report['recording_after'][k] for k in ('steps','events','metrics'))
    for category in ('graph','bullet','base_collision','regions','recording'):
        report[category+'_speedup'] = report[category+'_before']['seconds']/report[category+'_after']['seconds']
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline',default='4ef10ee')
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    result = json.dumps(run(args.baseline),indent=2)
    if args.output:
        args.output.write_text(result+'\n',encoding='utf-8')
    print(result)

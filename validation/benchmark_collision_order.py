"""Development comparison of collision order on identical full-case IK layers.

This is a benchmark, not a runtime strategy switch or planner option.
"""
import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from validate_mobile_base_case import load_case, setup
from motion_toolbox.geometry import as_plane
from motion_toolbox.mobile_base_workflow import generate_base_path
from motion_toolbox.planning import candidates, rotation_offsets
from motion_toolbox.graph import shortest_path
from motion_toolbox.recording import ResearchRun, event


def benchmark(folder, output):
    output.mkdir(parents=True, exist_ok=True)
    start = perf_counter()
    data = load_case(folder)
    replay = data['replay']
    report = dict(case_sha256=hashlib.sha256((folder/'case.json').read_bytes()).hexdigest())
    with ResearchRun(output/'research', name='full-case collision-order comparison',
                     config=dict(case=str(folder), rotation_steps=16, check_edges=False)) as run:
        targets = [as_plane(t) for t in replay['targets']]
        tick = perf_counter()
        proposal = generate_base_path(targets)
        bases = proposal['base_planes']
        report['geometry_seconds'] = perf_counter()-tick
        tick = perf_counter()
        solver, world = setup(folder, replay, calibrated=True)
        report['setup_seconds'] = perf_counter()-tick
        with world:
            tick = perf_counter()
            layers = []
            for i, (target, base) in enumerate(zip(targets, bases)):
                layers.append(candidates(target, base, solver, rotation_offsets('n_steps', steps=16),
                                         None, replay['joint_ranges'])[0])
                if (i+1)%100 == 0:
                    print('IK', i+1, '/', len(targets), flush=True)
            report['ik_seconds'] = perf_counter()-tick
            report['targets'] = len(targets)
            report['nodes'] = sum(map(len, layers))
            for mode in ('before_graph', 'after_graph'):
                tick = perf_counter()
                counts = dict(collision_checks=0, collision_rejections=0, graph_solves=0,
                              collision_seconds=0., graph_seconds=0.)
                cache = {}
                working = [list(qs) for qs in layers]
                def check(i, q):
                    key = i, tuple(q)
                    if key not in cache:
                        now = perf_counter()
                        cache[key] = world.is_valid(q, bases[i], clearance=replay['collision_options'].get('clearance', 0.))
                        counts['collision_seconds'] += perf_counter()-now
                        counts['collision_checks'] += 1
                        counts['collision_rejections'] += not cache[key]
                    return cache[key]
                if mode == 'before_graph':
                    working = [[q for q in qs if check(i, q)] for i, qs in enumerate(working)]
                while True:
                    now = perf_counter()
                    result = shortest_path(working, start=replay['current_pose'], periodic=replay['periodic'],
                                           max_step=replay['max_joint_step'], count_paths=False)
                    counts['graph_seconds'] += perf_counter()-now
                    counts['graph_solves'] += 1
                    if not result.configurations:
                        break
                    failures = [i for i, q in enumerate(result.configurations) if not check(i, q)]
                    if not failures:
                        break
                    for i in failures:
                        working[i] = [q for q in working[i] if cache.get((i, tuple(q)), True)]
                counts.update(elapsed_seconds=perf_counter()-tick, cost=result.cost if result.configurations else None,
                              valid=bool(result.configurations), failure_layer=result.failure_layer)
                report[mode] = counts
                event('benchmark.collision_order', mode=mode, **counts)
                print(mode, json.dumps(counts), flush=True)
        report['research_run'] = str(run.path)
    report['total_seconds'] = perf_counter()-start
    a, b = report['before_graph'], report['after_graph']
    assert a['valid'] == b['valid']
    if a['valid']:
        assert np.isclose(a['cost'], b['cost'], rtol=1e-12, atol=1e-12)
    report['selected'] = min(('before_graph', 'after_graph'), key=lambda m: report[m]['elapsed_seconds'])
    (output/'comparison.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    benchmark(args.case, args.output)

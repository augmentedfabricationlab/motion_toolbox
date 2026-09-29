"""Synthetic non-arc regression benchmark, not captured-robot validation.

Run with --output pointing outside Git. Each policy gets fresh IK/collision
caches and normal research recording. No saved trajectories are reused.
"""
import argparse
import json
from pathlib import Path
from time import perf_counter, process_time

from motion_toolbox import __version__
from motion_toolbox.geometry import Plane
from motion_toolbox.mobile_base_workflow import generate_base_path, validate_base_path
from motion_toolbox.recording import ResearchRun


class Solver:
    arm_in_base = Plane.world_xy()
    revolute_joints = ()

    def __call__(self, target, base):
        return [[j*.05, float(target.origin[0]), 0., 0., 0., 0.] for j in range(32)]


class World:
    last_failure = 'synthetic configuration rejection'

    def is_base_valid(self, base, **kwargs):
        return True

    def is_valid(self, q, base, **kwargs):
        return q[0] >= 31*.05

    def edge_is_valid(self, *args, **kwargs):
        raise AssertionError('Transition collision checks are not permitted')


def benchmark(output, count=872):
    output.mkdir(parents=True, exist_ok=True)
    report = dict(package_version=__version__, synthetic=True, targets=count,
                  candidates_per_target=32, recording='normal', trials=[])
    reference = None
    for complete in (False, True):
        started, cpu = perf_counter(), process_time()
        with ResearchRun(output/'research', name='non-arc failed-layer benchmark',
                         config=dict(complete_collision_layers=complete, targets=count)) as run:
            targets = [Plane((i*.002,0,1),(0,0,1),(1,0,0)) for i in range(count)]
            proposal = generate_base_path(targets, max_xy_deviation=.01)
            assert all(s['kind'] != 'arc' for s in proposal['path_sections'])
            result = validate_base_path(targets, proposal['base_planes'], solver=Solver(), world=World(),
                joint_ranges=[[-3,3]]*6, periodic=[False]*6, rotation_steps=16,
                _complete_collision_layers=complete)
            assert result['fabrication_validated'], result['status']
            selected = result['configurations'], result['path_length']
            if reference is None:
                reference = selected
            assert selected == reference
        trial = dict(policy=result['collision_failed_layer_policy'], research_run=str(run.path),
                     wall_seconds=perf_counter()-started, cpu_seconds=process_time()-cpu,
                     graph_stats=result['graph_stats'], timings=result['timings'],
                     collision_checks=sum(d['collision_checks'] for d in result['target_diagnostics']),
                     path_cost=result['path_length'], recording_failed=run.failed)
        report['trials'].append(trial)
        print(json.dumps(trial), flush=True)
    report['identical_configurations_and_cost'] = True
    (output/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--targets', type=int, default=872)
    args = parser.parse_args()
    benchmark(args.output, args.targets)

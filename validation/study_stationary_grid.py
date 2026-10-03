"""Isolated, resumable stationary placement study; never changes GH defaults.

Run with the main checkout's Python environment. Captured assets/results stay
outside Git. Each worker owns an independent detailed PyBullet world.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from functools import partial
import hashlib
import json
import math
import os
from pathlib import Path
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]
os.environ['TOOLBOX_RECORDING'] = '0'
import numpy as np

from motion_toolbox.geometry import Plane, as_plane
from motion_toolbox.stationary_region import StationaryRegion
from motion_toolbox.execution import performance_scope
from validation.validate_mobile_base_case import load_case, setup


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)


def stratified_grid(polygon, n=8, seed=20261003):
    """One jittered point in each of n x n row/column strata, inside a convex polygon."""
    polygon = np.asarray(polygon, float)
    if len(polygon) < 3 or n < 1:
        raise ValueError('Need a nonempty polygon and positive grid size')
    rng = random.Random(seed)
    low, high = polygon[:, 1].min(), polygon[:, 1].max()
    points = []
    for row in range(n):
        y = low + (row + rng.uniform(.2, .8)) * (high-low) / n
        xs = []
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            if min(a[1], b[1]) <= y < max(a[1], b[1]):
                xs.append(a[0] + (y-a[1])*(b[0]-a[0])/(b[1]-a[1]))
        if len(xs) < 2:
            raise ValueError('Degenerate polygon row')
        for col in range(n):
            x = min(xs) + (col + rng.uniform(.2, .8))*(max(xs)-min(xs))/n
            points.append([x, y])
    rng.shuffle(points)
    return points


def base_for_arm(point, heading, mount, height):
    c, s = math.cos(heading), math.sin(heading)
    x, y = mount.origin[:2]
    return Plane((point[0]-c*x+s*y, point[1]-s*x-c*y, height), (c,s,0), (-s,c,0))


def candidate_id(base):
    return hashlib.sha256(np.round(base.matrix, 10).tobytes()).hexdigest()[:16]


def propose(points, region, stage, yaw_steps=4):
    output = []
    for point in points:
        towards = region.center-np.asarray(point)
        facing = math.atan2(towards[1], towards[0])-math.atan2(region.mount.xaxis[1], region.mount.xaxis[0])
        for yaw in facing + np.arange(yaw_steps)*2*math.pi/yaw_steps:
            base = base_for_arm(point, yaw, region.mount, region.base_height)
            output.append(dict(id=candidate_id(base), stage=stage, arm_xy=list(map(float, point)),
                               yaw=float(yaw), base=base.to_data() if hasattr(base, 'to_data') else
                               dict(origin=base.origin.tolist(), x_axis=base.xaxis.tolist(), y_axis=base.yaxis.tolist()),
                               original_region_metrics=region.metrics(base)))
    return output


def score_rows(rows, preference='balanced'):
    """Coverage first; normalized min/mean product leaves the tradeoff explicit."""
    valid = [r for r in rows if not r.get('base_blocked') and not r.get('start_blocked')]
    max_min = max((r['minimum_solutions'] for r in valid), default=0) or 1
    max_mean = max((r['mean_solutions'] for r in valid), default=0) or 1
    weight = {'balanced': .5, 'total': .25, 'minimum': .75}[preference]
    for r in rows:
        a, b = r['minimum_solutions']/max_min, r['mean_solutions']/max_mean
        r['score'] = a**weight * b**(1-weight)
    return sorted(valid, key=lambda r: (-r['reachable_targets'], -r['score'],
        -r['total_solutions'], -r['minimum_solutions'], r['id']))


def refinement_parents(rows, region, preference='balanced'):
    ranked = score_rows(rows, preference)
    if not ranked:
        return []
    coverage = ranked[0]['reachable_targets']
    contenders = [r for r in ranked if r['reachable_targets'] == coverage]
    # Keep both count extremes, then both ends along the target cloud's long axis.
    _, _, vh = np.linalg.svd(region.points[:, :2]-region.center, full_matrices=False)
    axis = vh[0]
    choices = [ranked[0], max(contenders, key=lambda r: (r['total_solutions'], r['minimum_solutions'])),
               max(contenders, key=lambda r: (r['minimum_solutions'], r['total_solutions'])),
               min(contenders, key=lambda r: np.dot(r['arm_xy'], axis)),
               max(contenders, key=lambda r: np.dot(r['arm_xy'], axis))]
    unique = {}
    for r in choices:
        unique.setdefault(tuple(np.round(r['arm_xy'], 8)), r)
    return list(unique.values())


def refinement_points(parents, cell_size):
    # Full 3x3 neighbourhood; intentionally NOT clipped to the original region.
    return [np.asarray(r['arm_xy'])+np.asarray([dx,dy])*cell_size
            for r in parents for dx in (-1,0,1) for dy in (-1,0,1)]


_WORLD = _SOLVER = _REPLAY = _OUTPUT = None
_OFFSETS = None


def initialize(case, output, steps):
    global _WORLD, _SOLVER, _REPLAY, _OUTPUT, _OFFSETS
    from motion_toolbox.planning import rotation_offsets
    _REPLAY = load_case(case)['replay']
    # Stationary collision semantics: keep detailed base and GPS coverage.
    _REPLAY['collision_options'].update(exclude_gps=False, base_collision_model='detailed')
    _SOLVER, _WORLD = setup(case, _REPLAY, calibrated=True)
    _OUTPUT = Path(output)
    _OFFSETS = rotation_offsets('n_steps', steps=steps)
    import atexit
    atexit.register(_WORLD.close)


def evaluate(proposal):
    from motion_toolbox.planning import candidates
    started, cpu = time.perf_counter(), time.process_time()
    base = as_plane(proposal['base'])
    n = len(_REPLAY['targets'])
    clearance = _REPLAY['collision_options'].get('clearance', 0.)
    counts, layers = [], []
    stats_total = dict(ik_seconds=0., collision_seconds=0., collision_checks=0, raw_ik=0)
    with performance_scope() as policy:
        body_valid = _WORLD.is_base_valid(base, clearance=clearance)
        start_valid = body_valid and (_REPLAY['current_pose'] is None or
                                     _WORLD.is_valid(_REPLAY['current_pose'], base, clearance=clearance))
        failure = _WORLD.last_failure
        if start_valid:
            for i, target in enumerate(_REPLAY['targets']):
                stats = {}
                qs, _, _ = candidates(target, base, _SOLVER, _OFFSETS,
                    partial(_WORLD.is_valid, clearance=clearance), _REPLAY['joint_ranges'], stats=stats)
                counts.append(len(qs))
                layers.append(np.asarray(qs, dtype=float).reshape((-1,6)))
                for key in stats_total:
                    stats_total[key] += stats[key]
                if (i+1) % 100 == 0:
                    atomic_json(_OUTPUT/'workers'/('%s.json'%os.getpid()),
                                dict(candidate=proposal['id'], targets_done=i+1, targets_total=n,
                                     elapsed_seconds=time.perf_counter()-started))
        else:
            counts = [0]*n  # No admissible configurations for an invalid base/start.
    row = dict(proposal, base_blocked=not body_valid, start_blocked=body_valid and not start_valid,
        base_failure=failure if not start_valid else None, counts=counts,
        counts_complete=True, targets_ik_checked=n if start_valid else 0,
        reachable_targets=sum(c>0 for c in counts), minimum_solutions=min(counts),
        total_solutions=sum(counts), mean_solutions=sum(counts)/n,
        elapsed_seconds=time.perf_counter()-started, cpu_seconds=time.process_time()-cpu,
        execution_policy=policy, **stats_total)
    if layers:
        lengths = np.array(counts, dtype=np.int64)
        # All exact checked configurations retained for later path checks, no rerun.
        np.savez_compressed(_OUTPUT/'layers'/(proposal['id']+'.npz'),
                            counts=lengths, configurations=np.concatenate(layers))
    atomic_json(_OUTPUT/'candidates'/(proposal['id']+'.json'), row)
    return row


def baseline(case, output, steps, region):
    from motion_toolbox.base_planning import find_stationary_base
    initialize(case, output, steps)
    bases, _, _ = region.candidates()
    started = time.perf_counter()
    with performance_scope():
        result = find_stationary_base(_REPLAY['targets'], bases, _REPLAY['current_pose'],
            ik_solver=_SOLVER, objective='heuristic', placement_region=region,
            collision=_WORLD.is_valid, base_collision=_WORLD.is_base_valid,
            rotation_mode='n_steps', rotation_steps=steps, joint_ranges=_REPLAY['joint_ranges'],
            periodic=_REPLAY['periodic'], max_joint_step=_REPLAY['max_joint_step'],
            fast_validation=True, count_paths=False, max_validation_attempts=3)
    selected = result.base_plane if result.base_planes else result.heuristic_plane
    data = dict(elapsed_seconds=time.perf_counter()-started, selected_base=None,
                complete_path=bool(result.configurations), disconnected_detail=result.disconnected_detail,
                validation_attempts=result.validation_attempts, base_collision_checks=result.base_collision_checks)
    if selected is not None:
        data['selected_base'] = dict(origin=selected.origin.tolist(), x_axis=selected.xaxis.tolist(), y_axis=selected.yaxis.tolist())
        arm = region.metrics(selected)['arm_origin']
        proposal = dict(id=candidate_id(selected), stage='baseline', base=data['selected_base'],
                        arm_xy=arm[:2], yaw=math.atan2(selected.xaxis[1],selected.xaxis[0]),
                        original_region_metrics=region.metrics(selected))
        evaluate(proposal)
    _WORLD.close()
    atomic_json(Path(output)/'baseline.json', data)
    return data


def path_check(row, replay, output):
    from motion_toolbox.graph import shortest_path
    from motion_toolbox.base_planning import _stationary_disconnection
    saved = np.load(output/'layers'/(row['id']+'.npz'))
    layers = np.split(saved['configurations'], np.cumsum(saved['counts'])[:-1])
    started = time.perf_counter()
    with performance_scope():
        result = shortest_path(layers, start=replay['current_pose'], periodic=replay['periodic'],
            max_step=replay['max_joint_step'], count_paths=False, revolute_joints=list(range(6)))
    detail = dict(candidate=row['id'], path_complete=len(result.configurations)==len(layers),
        cost=result.cost if math.isfinite(result.cost) else None, failure_layer=result.failure_layer,
        seconds=time.perf_counter()-started, configurations=result.configurations,
        configuration_collision_checked=True, transition_collision_checked=False)
    atomic_json(output/'paths'/(row['id']+'.json'), detail)
    return {k:v for k,v in detail.items() if k!='configurations'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rotation-steps', type=int, default=24)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--seed', type=int, default=20261003)
    parser.add_argument('--grid-size', type=int, default=8)
    parser.add_argument('--refinement-rounds', type=int, default=2)
    parser.add_argument('--preference', choices=['balanced','total','minimum'], default='balanced')
    args = parser.parse_args()
    if args.rotation_steps<1 or args.workers<1 or args.grid_size<2 or args.refinement_rounds<0:
        parser.error('Positive samples/workers, grid-size >=2 and nonnegative rounds required')
    output = args.output.resolve()
    if output == ROOT or ROOT in output.parents:
        parser.error('Keep study results outside the source checkout')
    for name in ('candidates','layers','paths','workers'):
        (output/name).mkdir(parents=True, exist_ok=True)
    case = load_case(args.case)
    replay = case['replay']
    region = StationaryRegion(replay['targets'], replay['arm_in_base'], projected=True)
    poly, reason = region.polygon()
    if len(poly)<3:
        raise ValueError('No initial region: '+reason)
    polygon = poly+region.center
    from motion_toolbox import __version__
    from motion_toolbox.recording import loaded_versions
    settings = dict(schema='stationary-grid-study/1', package_version=__version__,
        case=str(args.case.resolve()), case_sha256=hashlib.sha256((args.case/'case.json').read_bytes()).hexdigest(),
        rotation_steps=args.rotation_steps, captured_rotation_steps=replay['rotation_steps'],
        grid_size=args.grid_size, seed=args.seed, yaw_steps=4, refinement_rounds=args.refinement_rounds,
        preference=args.preference, targets=len(replay['targets']), collision_model='detailed', exclude_gps=False,
        joint_ranges=replay['joint_ranges'], max_joint_step=replay['max_joint_step'],
        transition_collision_checked=False, captured_check_edges=replay.get('check_edges'),
        polygon=polygon.tolist(), arm_origin_height=region.height,
        code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if (output/'settings.json').exists():
        if json.loads((output/'settings.json').read_text()) != settings:
            raise ValueError('Resume settings/source differ; choose a fresh output directory')
    else:
        atomic_json(output/'settings.json', settings)
        atomic_json(output/'loaded_code.json', loaded_versions())
    def progress(**details):
        atomic_json(output/'progress.json', details)
        print(json.dumps(details), flush=True)
    progress(stage='baseline')
    base = json.loads((output/'baseline.json').read_text()) if (output/'baseline.json').exists() else baseline(args.case, output, args.rotation_steps, region)
    rows = {p.stem:json.loads(p.read_text()) for p in (output/'candidates').glob('*.json')}
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=args.workers, initializer=initialize,
                             initargs=(args.case,output,args.rotation_steps)) as pool:
        points = stratified_grid(polygon, args.grid_size, args.seed)
        cell = (polygon.max(0)-polygon.min(0))/args.grid_size
        for round_index in range(args.refinement_rounds+1):
            stage = 'coarse' if round_index==0 else 'refine_%s'%round_index
            if round_index:
                prior = [r for r in rows.values() if r['stage'] in ['baseline','coarse']+
                         ['refine_%s'%i for i in range(1,round_index)]]
                parents = refinement_parents(prior, region, args.preference)
                points = refinement_points(parents, cell/(2**(round_index-1)))
                atomic_json(output/(stage+'_parents.json'), [r['id'] for r in parents])
            proposals = {r['id']:r for r in propose(points,region,stage)}
            pending = [p for key,p in proposals.items() if key not in rows]
            futures = {pool.submit(evaluate,p):p for p in pending}
            progress(stage=stage, submitted=len(pending), already_done=len(proposals)-len(pending), completed=len(rows))
            for future in as_completed(futures):
                r = future.result(); rows[r['id']] = r
                ranked = score_rows(list(rows.values()),args.preference)
                progress(stage=stage, completed=len(rows), pending=sum(not f.done() for f in futures),
                    full_coverage=sum(r['reachable_targets']==len(replay['targets']) for r in rows.values()),
                    best={k:ranked[0][k] for k in ('id','reachable_targets','minimum_solutions','total_solutions')} if ranked else None,
                    study_elapsed_seconds=time.perf_counter()-started)
    ranked = score_rows(list(rows.values()),args.preference)
    finalists = {r['id']:r for r in ranked[:3]+refinement_parents(list(rows.values()),region,args.preference)}
    baseline_rows = [r for r in rows.values() if r['stage']=='baseline']
    finalists.update({r['id']:r for r in baseline_rows})
    paths=[]
    for r in finalists.values():
        if r['reachable_targets'] == len(replay['targets']):
            progress(stage='path_check', candidate=r['id'])
            path=output/'paths'/(r['id']+'.json')
            if path.exists():
                detail=json.loads(path.read_text()); paths.append({k:v for k,v in detail.items() if k!='configurations'})
            else:
                paths.append(path_check(r,replay,output))
    result=dict(settings=settings,baseline=base,candidates=list(rows.values()),
                ranked_ids=[r['id'] for r in ranked],paths=paths,status='complete',
                search_seconds_this_invocation=time.perf_counter()-started)
    atomic_json(output/'result.json',result)
    progress(stage='complete', candidates=len(rows), connected_finalists=sum(p['path_complete'] for p in paths))


if __name__ == '__main__':
    main()

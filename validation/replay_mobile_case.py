"""Replay an exported mobile case in a killable subprocess, without Rhino.

Usage: python validation/replay_mobile_case.py CASE --output OUT --timeout 180
Optional --settings JSON overrides mobile proposal settings only.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))


def load_case(folder):
    folder = Path(folder).resolve()
    if not (folder / 'READY').is_file():
        raise ValueError('Capture is not READY')
    manifest = json.loads((folder / 'manifest.json').read_text())
    if not {'case.json','robot/robot.urdf'} <= set(manifest):
        raise ValueError('Manifest must include case.json and robot/robot.urdf')
    for name, expected in manifest.items():
        path = (folder / name).resolve()
        if folder not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Capture integrity failure: ' + name)
    data = json.loads((folder / 'case.json').read_text())
    if data['schema'] != 'motion-mobile-case/1' or data['replay']['units'] != 'metres/radians':
        raise ValueError('Unsupported capture schema or units')
    return data


def setup(folder, replay, calibrated=False):
    from motion_toolbox.collision import PybulletServer
    from motion_toolbox.kinematics.solver import URKinematics
    settings = replay['collision_options']
    world = PybulletServer(Path(folder) / 'robot/robot.urdf',
        joint_names=replay['arm_joint_names'],
        **{k: settings[k] for k in ('allowed_pairs', 'check_static_self_collisions') if k in settings})
    try:
        world.set_fixed_joints(replay['fixed_joint_values'])
        for mesh in replay['environment_meshes']:
            world.add_mesh((mesh['vertices'], mesh['faces']))
        for tool in replay['tool_collisions']:
            mesh = tool['mesh']
            world.attach_mesh((mesh['vertices'], mesh['faces']), tool['link_name'],
                              frame=tool['frame'], touch_links=tool['touch_links'])
        if 'ground_z' in settings:
            world.add_ground(settings['ground_z'], support_links=settings.get('support_links', ()))
        solver = URKinematics(replay['ur_parameters'], tool=replay['tcp_in_flange'],
                              arm_in_base=replay['arm_in_base'])
        if calibrated:
            from motion_toolbox.kinematics.calibrated import CalibratedURKinematics
            prefix = replay['arm_joint_names'][0][:-len('shoulder_pan_joint')]
            solver = CalibratedURKinematics((Path(folder)/'robot/robot.urdf').read_text(),
                replay['arm_joint_names'], controller_link=prefix+'base', end_link=prefix+'tool0',
                fixed_joint_values=replay['fixed_joint_values'], parameters=replay['ur_parameters'],
                tool=replay['tcp_in_flange'], arm_in_base=replay['arm_in_base'])
        return solver, world
    except BaseException:
        world.close()
        raise


def audit_forward_kinematics(replay, solver, world, targets):
    """Independent captured-URDF FK audit; analytic IK success is insufficient."""
    import numpy as np
    from motion_toolbox.geometry import as_plane, Plane
    from motion_toolbox.smooth_mobile import smooth_offset_proposals
    proposals, _ = smooth_offset_proposals(targets, solver.arm_in_base,
        windows=[100], wall_distances=[1.], lateral_offsets=[1.])
    if not proposals:
        return dict(status='no_audit_proposal')
    suffix = 'shoulder_pan_joint'
    name = replay['arm_joint_names'][0]
    link = name[:-len(suffix)]+'tool0' if name.endswith(suffix) else None
    if link not in world.links:
        return dict(status='unresolved_tcp_attachment')
    records = []
    for i in sorted({0, min(57,len(targets)-1), min(1000,len(targets)-1)}):
        base = proposals[0][1][i]
        rows = solver(targets[i], base)
        if not rows:
            records.append(dict(target=i, status='no_ik'))
            continue
        world.set_base(base)
        for joint,value in zip(world.joints,rows[0]):
            world.p.resetJointState(world.robot,joint,value)
        state = world.p.getLinkState(world.robot,world.links[link],computeForwardKinematics=True)
        actual = np.eye(4)
        actual[:3,:3] = np.asarray(world.p.getMatrixFromQuaternion(state[5])).reshape(3,3)
        actual[:3,3] = state[4]
        actual = actual @ as_plane(replay['tcp_in_flange']).matrix
        cosine = np.clip((np.trace(actual[:3,:3].T @ targets[i].matrix[:3,:3])-1)/2,-1,1)
        records.append(dict(target=i, position_error_metres=float(np.linalg.norm(actual[:3,3]-targets[i].origin)),
                            orientation_error_radians=float(np.arccos(cosine)), joints=rows[0], base=base.to_dict()))
    return dict(status='sampled_only', reference='captured URDF tool0 plus captured TCP', samples=records)


def worker(args):
    # The harness persists its own inputs/fingerprints/results. Avoid creating
    # a separate automatic research recording for each scene construction call.
    os.environ['TOOLBOX_RECORDING'] = '0'
    source_root = (Path(args.case)/'source' if args.captured_source else Path(__file__).resolve().parents[1]/'src')
    sys.path.insert(0, str(source_root.resolve()))
    source_hashes = {str(p.relative_to(source_root)):hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in source_root.rglob('*.py')}
    import faulthandler
    faulthandler.dump_traceback_later(60, repeat=True)
    from functools import partial
    import numpy as np
    from motion_toolbox.geometry import as_plane, Plane
    from motion_toolbox.mobile_planning import plan_mobile_robot_path
    started = time.perf_counter()
    cpu_started = time.process_time()
    data = load_case(args.case)
    r = data['replay']
    solver, world = setup(args.case, r, calibrated=not args.captured_source)
    settings = dict(r['mobile_options'], **json.loads(args.settings))
    targets = [as_plane(t) for t in r['targets']]
    seeds = [as_plane(b) for b in r['seeds']]
    if args.target_count:
        targets = targets[:args.target_count]
        seeds = seeds[:args.target_count] if len(seeds)>1 else seeds
        if 'time_intervals' in settings:
            settings['time_intervals'] = settings['time_intervals'][:len(targets)-int(r['current_pose'] is None)]
    extra = {}
    if args.proposal_file:
        from motion_toolbox.stationary_region import StationaryRegion
        seeds = [as_plane(b) for b in json.loads(Path(args.proposal_file).read_text())]
        if args.target_count:
            seeds = seeds[:args.target_count]
        if len(seeds) != len(targets):
            raise ValueError('Proposal must have one base per original target')
        limits = {k:v for k,v in settings.items() if k in ('start_base','max_base_step','max_yaw_step',
            'time_intervals','max_base_speed','max_yaw_speed','max_joint_speed')}
        settings = dict(strategy='discrete', sparse=False, placement_region=False, **limits)
        def valid(target, base):
            return (StationaryRegion([target], solver.arm_in_base, max_distance=1.75,
                base_height=r['mobile_options'].get('base_height',0.), projected=True).metrics(base)['geometry_valid']
                and (not r['collision_check'] or world.is_base_valid(base, clearance=r['collision_options'].get('clearance',0.))))
        extra['base_valid'] = valid
    print('Loaded verified capture: {} targets, {} obstacles, {} tools'.format(
        len(targets), len(world.environment), len(world.tools)), flush=True)
    import motion_toolbox.base_planning as bp
    original_candidates = bp.candidates
    evaluated = 0
    def progress(*a, **kw):
        nonlocal evaluated
        value = original_candidates(*a, **kw)
        evaluated += 1
        if evaluated % 100 == 0 or not value[0]:
            print('Candidate evaluations {}, elapsed {:.2f}s, states {}, details {}'.format(
                evaluated, time.perf_counter()-started, len(value[0]), kw.get('stats')), flush=True)
        return value
    bp.candidates = progress
    with world:
        audit = audit_forward_kinematics(r, solver, world, targets)
        (Path(args.output)/'fk_audit.json').write_text(json.dumps(audit, indent=2))
        c = r['collision_options']
        clearance = c.get('clearance', 0.)
        result = plan_mobile_robot_path(targets, seeds, settings, **extra,
            ik_solver=solver, current_pose=r['current_pose'], joint_ranges=r['joint_ranges'],
            periodic=r['periodic'], max_joint_step=r['max_joint_step'], rotation_steps=r['rotation_steps'],
            base_collision=partial(world.is_base_valid, clearance=clearance) if r['collision_check'] else None,
            collision=partial(world.is_valid, clearance=clearance) if r['collision_check'] else None,
            transition_check=partial(world.edge_is_valid, periodic=r['periodic'], clearance=clearance,
                **{k:c[k] for k in ('joint_resolution','base_resolution','yaw_resolution') if k in c})
                if r['collision_check'] and r['check_edges'] else None)
    result['replay_metadata'] = dict(settings=settings, elapsed_seconds=time.perf_counter()-started,
        cpu_seconds=time.process_time()-cpu_started,
        case_sha256=hashlib.sha256((Path(args.case)/'case.json').read_bytes()).hexdigest(),
        source_sha256_at_start=source_hashes,
        source_changed_during_run=any(hashlib.sha256((source_root/p).read_bytes()).hexdigest()!=h
                                     for p,h in source_hashes.items()),
        executable=sys.executable, fixed_joint_values=r['fixed_joint_values'],
        arm_joint_names=r['arm_joint_names'], collision_options=c)
    result['replay_metadata'].update(original_target_count=len(r['targets']), tested_target_count=len(targets),
        complete_original_path=len(result['configurations']) == len(r['targets']))
    result['forward_kinematics_audit'] = audit
    from motion_toolbox.recording import loaded_versions
    from motion_toolbox import __version__
    result['replay_metadata'].update(toolbox_version=__version__, loaded_versions=loaded_versions())
    def encode(obj):
        if isinstance(obj, Plane): return obj.to_dict()
        if isinstance(obj, np.ndarray): return obj.tolist()
        if isinstance(obj, np.generic): return obj.item()
        raise TypeError(type(obj).__name__)
    (Path(args.output)/'result.json').write_text(json.dumps(result, default=encode, indent=2))
    print('Finished: {} configurations, {:.3f}s'.format(len(result['configurations']),
        result['replay_metadata']['elapsed_seconds']), flush=True)
    faulthandler.cancel_dump_traceback_later()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case')
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--settings', default='{}')
    parser.add_argument('--settings-file')
    parser.add_argument('--proposal-file')
    parser.add_argument('--target-count', type=int)
    parser.add_argument('--captured-source', action='store_true')
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.timeout <= 0 or (args.target_count is not None and args.target_count < 1):
        parser.error('Timeout and target count must be positive')
    if args.settings_file:
        args.settings = Path(args.settings_file).read_text(encoding='utf-8-sig')
    json.loads(args.settings)
    Path(args.output).mkdir(parents=True, exist_ok=True)
    if args.worker:
        worker(args)
        return
    command = [sys.executable, str(Path(__file__).resolve()), args.case, '--output', args.output,
               '--settings', args.settings, '--worker']
    if args.proposal_file:
        command.extend(['--proposal-file', str(Path(args.proposal_file).resolve())])
    if args.target_count:
        command.extend(['--target-count', str(args.target_count)])
    if args.captured_source:
        command.append('--captured-source')
    started = time.perf_counter()
    with (Path(args.output)/'worker.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
        try:
            code = process.wait(timeout=args.timeout)
            status = 'finished' if code == 0 else 'error'
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            code, status = None, 'timeout'
    report = dict(status=status, exit_code=code, elapsed_seconds=time.perf_counter()-started,
                  timeout_seconds=args.timeout, command=command)
    (Path(args.output)/'run.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report))


if __name__ == '__main__':
    main()

"""Replay captured adaptive mobile planning offline, without a runtime limit."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))

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

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rotation-steps',type=int,default=16)
    parser.add_argument('--fixed-offsets',action='store_true')
    parser.add_argument('--worker',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if not args.worker:
        started=time.perf_counter()
        command=[sys.executable,__file__,str(args.case),'--output',str(args.output),'--rotation-steps',str(args.rotation_steps),'--worker']
        if args.fixed_offsets:command.append('--fixed-offsets')
        with (args.output/'worker.log').open('w') as log:
            run=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        if run.returncode==0:
            path=args.output/'result.json'
            result=json.loads(path.read_text())
            result['run']['end_to_end_seconds']=time.perf_counter()-started
            path.write_text(json.dumps(result,indent=2))
            print(result['status'],result['run']['end_to_end_seconds'],flush=True)
        raise SystemExit(run.returncode)
    import numpy as np
    import sqlite3
    from motion_toolbox.geometry import Plane,as_plane
    from motion_toolbox.mobile_base_workflow import plan_base_path
    from motion_toolbox.recording import ResearchRun,recorded,event
    from motion_toolbox import __version__
    import motion_toolbox.mobile_base_workflow as module
    start=time.perf_counter();cpu=time.process_time()
    case_hash=hashlib.sha256((args.case/'case.json').read_bytes()).hexdigest()
    def progress(message):
        (args.output/'progress.json').write_text(json.dumps(message,indent=2))
        print(json.dumps(message),flush=True)
    @recorded
    def execute():
        data=load_case(args.case);r=data['replay']
        event('capture.verified',case_sha256=case_hash,targets=len(r['targets']),
              calibration=dict(arm_in_base=r['arm_in_base'],tcp_in_flange=r['tcp_in_flange']),
              collision_options=r['collision_options'],fixed_joint_values=r['fixed_joint_values'])
        targets=[as_plane(t) for t in r['targets']]
        tick=time.perf_counter()
        solver,world=setup(args.case,r,calibrated=True)
        setup_seconds=time.perf_counter()-tick
        with world:
            result=plan_base_path(targets,solver=solver,world=world,
                joint_ranges=r['joint_ranges'],periodic=r['periodic'],current_pose=r['current_pose'],
                progress=progress,adapt_offsets=not args.fixed_offsets,rotation_steps=args.rotation_steps,
                max_joint_step=r['max_joint_step'],collision_options=r['collision_options'],
                **{k:v for k,v in r.get('mobile_options',{}).items() if k in
                   ('max_base_step','max_yaw_step','time_intervals','max_base_speed','max_yaw_speed','max_joint_speed')})
        result['timings']['setup_seconds']=setup_seconds
        result['run']=dict(case_sha256=case_hash,version=__version__,
            source_sha256=hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest(),
            python=sys.version,executable=sys.executable,
            source_hashes={str(p.relative_to(Path(module.__file__).parent)):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in Path(module.__file__).parent.rglob('*.py')},
            collision_options=r['collision_options'],arm_in_base=r['arm_in_base'],tcp_in_flange=r['tcp_in_flange'],
            fixed_joint_values=r['fixed_joint_values'],normal_recording=True)
        return result
    with ResearchRun(args.output/'research',name='mobile capture replay',
                     config=dict(case=str(args.case),case_sha256=case_hash,rotation_steps=args.rotation_steps,
                                 adapt_offsets=not args.fixed_offsets,check_edges=False)) as run:
        result=execute()
    result['research_run']=str(run.path)
    with sqlite3.connect(str(run.path/'run.sqlite3')) as db:
        result['timings']['recording_overhead_seconds']=db.execute(
            "SELECT COALESCE(SUM(value),0)/1e9 FROM metrics WHERE name='recording_overhead'").fetchone()[0]
    result['run'].update(elapsed_seconds=time.perf_counter()-start,cpu_seconds=time.process_time()-cpu)
    def encode(x):
        if isinstance(x,Plane):return x.to_dict()
        if isinstance(x,np.ndarray):return x.tolist()
        if isinstance(x,np.generic):return x.item()
        raise TypeError(type(x).__name__)
    (args.output/'result.json').write_text(json.dumps(result,default=encode,indent=2,allow_nan=False))
    print(result['status'],result['state_counts'],result['run']['elapsed_seconds'],flush=True)

if __name__=='__main__':main()

"""Independently audit every saved target/transition using captured URDF FK.

Run this CLI in a bounded subprocess, as with replay_mobile_case.py --worker.
"""
import argparse
from functools import partial
import hashlib
import json
import os
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
os.environ['TOOLBOX_RECORDING'] = '0'
import numpy as np
from replay_mobile_case import load_case, setup
from motion_toolbox.geometry import as_plane
from motion_toolbox.mobile_transitions import MobileTransitions
from motion_toolbox.stationary_region import StationaryRegion


def audit(folder,result):
    started = time.perf_counter()
    r = load_case(folder)['replay']
    solver,world = setup(folder,r,calibrated=True)
    targets = [as_plane(t) for t in r['targets']]
    bases = [as_plane(b) for b in result['base_planes']]
    qs = np.asarray(result['configurations'],dtype=float)
    if len(bases)!=len(targets) or qs.shape!=(len(targets),len(r['arm_joint_names'])) or not np.isfinite(qs).all():
        world.close()
        raise ValueError('Saved result must contain one finite base/configuration per original target')
    settings = dict(r['mobile_options'],**result.get('replay_metadata',{}).get('settings',{}))
    constraints = dict(max_base_step=.25,max_yaw_step=.25,max_joint_step=r['max_joint_step'],
        max_base_speed=None,max_yaw_speed=None,max_joint_speed=None,time_intervals=None,
        start_base=None,periodic=r['periodic'])
    constraints.update({k:v for k,v in settings.items() if k in constraints})
    collisions = r['collision_options']
    clearance = collisions.get('clearance',0.)
    edge = partial(world.edge_is_valid,periodic=r['periodic'],clearance=clearance,
        **{k:v for k,v in collisions.items() if k in ('joint_resolution','base_resolution','yaw_resolution')})
    transition = MobileTransitions(dict(constraints,transition_check=edge if r['collision_check'] and r['check_edges'] else None))
    errors,reach,rejections = [],[],[]
    prefix = r['arm_joint_names'][0][:-len('shoulder_pan_joint')]
    link = world.links[prefix+'tool0']
    with world:
        for i,(target,base,q) in enumerate(zip(targets,bases,qs)):
            region = StationaryRegion([target],solver.arm_in_base,max_distance=1.75,
                base_height=settings.get('base_height',0.),projected=True)
            if not region.metrics(base)['geometry_valid']:
                rejections.append(dict(target=i,reason='placement'))
            if not np.allclose(base.zaxis,[0,0,1],atol=1e-9) or abs(base.origin[2]-settings.get('base_height',0.))>1e-9:
                rejections.append(dict(target=i,reason='ground_plane'))
            for j,(value,limits) in enumerate(zip(q,r['joint_ranges'])):
                if limits and ((limits[0] is not None and value<limits[0]) or (limits[1] is not None and value>limits[1])):
                    rejections.append(dict(target=i,joint=j,reason='joint_limit',measured=float(value),limit=limits))
            if r['collision_check']:
                if not world.is_base_valid(base,clearance=clearance):
                    rejections.append(dict(target=i,reason=world.last_failure))
                if not world.is_valid(q,base,clearance=clearance):
                    rejections.append(dict(target=i,reason=world.last_failure))
            else:
                world.set_base(base)
                for joint,value in zip(world.joints,q):
                    world.p.resetJointState(world.robot,joint,value)
            # Independent Bullet link FK, not the refinement solver's FK.
            state = world.p.getLinkState(world.robot,link,computeForwardKinematics=True)
            actual = np.eye(4)
            actual[:3,:3] = np.asarray(world.p.getMatrixFromQuaternion(state[5])).reshape(3,3)
            actual[:3,3] = state[4]
            actual = actual@solver.tool.matrix
            position = float(np.linalg.norm(actual[:3,3]-target.origin))
            rotation = float(np.arccos(np.clip((np.trace(actual[:3,:3].T@target.matrix[:3,:3])-1)/2,-1,1)))
            errors.append((position,rotation))
            if position>1e-6 or rotation>1e-6:
                rejections.append(dict(target=i,reason='independent_fk',position_metres=position,rotation_radians=rotation,
                                       tolerance=1e-6))
            arm = (base.matrix@solver.arm_in_base.matrix)[:3,3]
            reach.append(float(np.linalg.norm(target.origin[:2]-arm[:2])))
            previous = ((qs[i-1],bases[i-1]) if i else
                (r['current_pose'],as_plane(settings['start_base'])) if r['current_pose'] is not None else None)
            if previous is not None:
                transition.reset()
                if next(transition.reachable(i,[previous],q,base),None) is None:
                    rejections.append(transition.diagnostic(i))
    positions = np.array([b.origin for b in bases])
    yaw = np.unwrap([np.arctan2(b.xaxis[1],b.xaxis[0]) for b in bases])
    steps = np.linalg.norm(np.diff(positions,axis=0),axis=1)
    roughness = np.linalg.norm(np.diff(positions,n=2,axis=0),axis=1)
    return dict(valid=not rejections,target_count=len(targets),configuration_count=len(qs),
        checked_transitions=len(targets)-1+int(r['current_pose'] is not None),rejections=rejections,
        max_fk_position_error_metres=max(e[0] for e in errors),max_fk_orientation_error_radians=max(e[1] for e in errors),
        fk_reference='independent PyBullet captured-URDF tool0 FK plus captured TCP',fk_tolerance=1e-6,
        max_arm_xy_reach=max(reach),minimum_xy_reach_margin=1.75-max(reach),
        max_base_step=float(steps.max(initial=0)),max_yaw_step=float(abs(np.diff(yaw)).max(initial=0)),
        max_joint_step=float(abs(np.diff(qs,axis=0)).max(initial=0)),
        mean_xy_second_difference=float(roughness.mean()) if len(roughness) else 0.,
        max_xy_second_difference=float(roughness.max(initial=0)),total_base_travel=float(steps.sum()),
        constraints=constraints,collision_check=r['collision_check'],check_edges=r['check_edges'],
        collision_options=collisions,elapsed_seconds=time.perf_counter()-started)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case')
    parser.add_argument('result')
    parser.add_argument('--output',required=True)
    args = parser.parse_args()
    report = audit(args.case,json.loads(Path(args.result).read_text()))
    report['result_sha256'] = hashlib.sha256(Path(args.result).read_bytes()).hexdigest()
    report['case_sha256'] = hashlib.sha256((Path(args.case)/'case.json').read_bytes()).hexdigest()
    from motion_toolbox import __version__
    from motion_toolbox.recording import loaded_versions
    report.update(toolbox_version=__version__,loaded_versions=loaded_versions(),
                  auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    Path(args.output).write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('collision_options','loaded_versions')}))
    if not report['valid']:
        raise SystemExit(1)


if __name__=='__main__':
    main()

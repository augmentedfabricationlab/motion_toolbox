"""Run the experimental fast search on a captured case, with reproducible output."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from functools import partial
from time import perf_counter

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT)]
os.environ['TOOLBOX_RECORDING']='0'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--full-checks',type=int,default=3)
    parser.add_argument('--rotation-steps',type=int,default=24)
    parser.add_argument('--tool-clearance',type=float,default=1.)
    parser.add_argument('--strategy',choices=('adaptive','heuristic'),default='adaptive')
    args=parser.parse_args()
    from validation.study_stationary_grid import load_case,setup,atomic_json
    from motion_toolbox.adaptive_stationary import find_adaptive_stationary_base,footprint_clearance
    from motion_toolbox.recording import loaded_versions
    from motion_toolbox import __version__
    from motion_toolbox.execution import performance_scope
    output=args.output.resolve()
    if ROOT==output or ROOT in output.parents:parser.error('Use an output directory outside the checkout')
    output.mkdir(parents=True,exist_ok=True)
    started=perf_counter();replay=load_case(args.case)['replay']
    replay['collision_options'].update(exclude_gps=False,base_collision_model='detailed')
    with performance_scope() as execution_policy:
        solver,world=setup(args.case,replay,calibrated=True)
        setup_seconds=perf_counter()-started
        try:
            def progress(message):
                try:atomic_json(output/'progress.json',message)
                except PermissionError:pass  # A locked preview must not abort computation.
                print(json.dumps(message),flush=True)
            common=dict(ik_solver=solver,
                collision=partial(world.is_valid,clearance=replay['collision_options'].get('clearance',0.)),
                base_collision=partial(world.is_base_valid,clearance=replay['collision_options'].get('clearance',0.)),
                joint_ranges=replay['joint_ranges'],periodic=replay['periodic'],
                rotation_steps=args.rotation_steps,max_joint_step=replay['max_joint_step'],progress=progress)
            if args.strategy=='adaptive':
                result=find_adaptive_stationary_base(replay['targets'],**common,
                    arm_in_base=replay['arm_in_base'],
                    clearance_measure=footprint_clearance(world,replay['targets']),tool_clearance=args.tool_clearance,
                    current_pose=replay['current_pose'],max_full_checks=args.full_checks)
            else:
                from motion_toolbox.base_planning import find_stationary_base
                from motion_toolbox.stationary_region import StationaryRegion
                from motion_toolbox.adaptive_stationary import _Evaluation
                class Counter(_Evaluation):
                    def __call__(self,target,base):
                        self.ik_cache.clear()
                        return super().__call__(target,base)
                    def valid(self,q,base):
                        self.collision_cache.clear()
                        return super().valid(q,base)
                counter=Counter(solver,common['collision'],None)
                common.update(ik_solver=counter,collision=counter.valid)
                region=StationaryRegion(replay['targets'],replay['arm_in_base'],max_distance=1.75,projected=True)
                bases,_,_=region.candidates(spacing=.5,yaw_steps=4)
                search_start=perf_counter()
                found=find_stationary_base(replay['targets'],bases,replay['current_pose'],**common,
                    objective='heuristic',placement_region=region,fast_validation=True,count_paths=False,
                    rotation_mode='n_steps',max_validation_attempts=args.full_checks)
                result=dict(found=found,status='connected' if found.configurations else 'no_connected_path',
                    stats=dict(counter.stats,total_seconds=perf_counter()-search_start,
                        graph_seconds=sum(d.get('timings',{}).get('path_seconds',0.) for d in found.diagnostics),
                        full_checks=found.validation_attempts),candidate_count=len(bases))
            found=result.pop('found')
            result['path_complete']=len(found.configurations)==len(replay['targets'])
            result['base']=found.base_plane.to_dict() if found.base_planes else None
            result['configurations']=found.configurations
            result['cost']=found.cost if result['path_complete'] else None
            result['selected_tcp_rotations']=found.selected_tcp_rotations
            result['disconnected_detail']=found.disconnected_detail
            result['validation_diagnostics']=[{k:d.get(k) for k in
                ('reason','unreachable_points','failed_target_details','timings')} for d in found.diagnostics]
            result['planning_total_seconds']=perf_counter()-started
            # Reopen the captured scene, independently replay collisions and verify
            # calibrated FK against every selected TCP rotation, limits and steps.
            verify=perf_counter()
            result['final_collision_recheck']=False
            if result['path_complete']:
                import numpy as np
                from motion_toolbox.geometry import as_plane
                from motion_toolbox.planning import _in_ranges
                fresh_solver,fresh_world=setup(args.case,replay,calibrated=True)
                try:
                    result['final_collision_recheck']=all(fresh_world.is_valid(q,found.base_plane,
                        clearance=replay['collision_options'].get('clearance',0.)) for q in found.configurations)
                    actual=[found.base_plane.matrix@fresh_solver.arm_in_base.matrix@fresh_solver.forward(q)@fresh_solver.tool.matrix
                            for q in found.configurations]
                    expected=[as_plane(t).rotated_z(a).matrix for t,a in zip(replay['targets'],found.selected_tcp_rotations)]
                    result['max_fk_matrix_error']=float(np.max(np.abs(np.array(actual)-expected)))
                    result['joint_limits_recheck']=all(_in_ranges(q,replay['joint_ranges']) for q in found.configurations)
                    qs=found.configurations if replay['current_pose'] is None else [replay['current_pose']]+found.configurations
                    delta=np.diff(qs,axis=0)
                    mask=np.zeros(delta.shape[1],bool) if replay['periodic'] is None else np.asarray(replay['periodic'],bool)
                    delta[:,mask]=(delta[:,mask]+np.pi)%(2*np.pi)-np.pi
                    result['joint_steps_recheck']=bool(np.all(abs(delta)<=np.asarray(replay['max_joint_step'])+1e-9))
                    if not all((result['final_collision_recheck'],result['joint_limits_recheck'],result['joint_steps_recheck'],
                                result['max_fk_matrix_error']<1e-6)):
                        raise AssertionError('Independent path verification failed')
                finally:fresh_world.close()
            result['verification_seconds']=perf_counter()-verify
            result['setup_seconds']=setup_seconds
            result['total_seconds']=perf_counter()-started
            result['package_version']=__version__
            result['loaded_code']=loaded_versions()
            result['case']=str(args.case.resolve())
            result['case_sha256']=hashlib.sha256((args.case/'case.json').read_bytes()).hexdigest()
            result['rotation_steps']=args.rotation_steps
            result['strategy']=args.strategy
            result['tool_clearance_preference_metres']=args.tool_clearance
            result['transition_collision_checked']=False
        finally:world.close()
    result['execution_policy']=dict(execution_policy)
    atomic_json(output/'result.json',result)
    print(json.dumps({k:result[k] for k in ('status','path_complete','cost','total_seconds','stats')}),flush=True)


if __name__=='__main__':main()

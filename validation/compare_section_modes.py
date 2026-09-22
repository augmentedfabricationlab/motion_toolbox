"""Replay straight captures through both geometry modes in one unchanged world.

Exact target/base/configuration caches are shared only after checking bit-identical
base matrices. This avoids duplicate calibrated IK without using saved solutions.
"""
import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from validate_mobile_base_case import load_case,setup
from motion_toolbox.geometry import Plane,as_plane
from motion_toolbox.mobile_base_workflow import generate_base_path,validate_base_path
from motion_toolbox.execution import high_qos
from motion_toolbox.recording import ResearchRun


@high_qos
def compare(case, normal_offset=.9, tangent_offset=1.2):
    start=perf_counter()
    replay=load_case(case)['replay']
    targets=[as_plane(t) for t in replay['targets']]
    proposals={mode:generate_base_path(targets,geometry_mode=mode,
        normal_offset=normal_offset,tangent_offset=tangent_offset) for mode in ('legacy','auto')}
    np.testing.assert_array_equal([b.matrix for b in proposals['legacy']['base_planes']],
                                  [b.matrix for b in proposals['auto']['base_planes']])
    solver,world=setup(case,replay,True)
    cache={}
    results={}
    with world:
        for mode in ('legacy','auto'):
            results[mode]=validate_base_path(targets,proposals[mode]['base_planes'],solver=solver,world=world,
                joint_ranges=replay['joint_ranges'],periodic=replay['periodic'],current_pose=replay['current_pose'],
                rotation_steps=16,max_joint_step=replay['max_joint_step'],collision_options=replay['collision_options'],
                _cache=cache,progress=lambda d:print(mode,json.dumps(d),flush=True))
            print(mode,results[mode]['status'],flush=True)
        assert results['legacy']['fabrication_validated'] and results['auto']['fabrication_validated']
        assert results['legacy']['configurations']==results['auto']['configurations']
        assert results['legacy']['path_length']==results['auto']['path_length']
        excluded=sorted(world.excluded_collision_links)
    return dict(case=str(case),case_sha256=hashlib.sha256((case/'case.json').read_bytes()).hexdigest(),
        targets=len(targets),poses_identical=True,configurations_identical=True,cost_identical=True,
        normal_offset=normal_offset,tangent_offset=tangent_offset,
        cost=results['auto']['path_length'],excluded_collision_links=excluded,
        path_sections=proposals['auto']['path_sections'],elapsed_seconds=perf_counter()-start,
        cache_policy='Full-precision per-call cache in an unchanged world; no saved trajectories',
        results=results)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--normal-offset',type=float,default=.9)
    parser.add_argument('--tangent-offset',type=float,default=1.2)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    with ResearchRun(args.output/'research',name='auto/legacy straight regression',
                     config=dict(case=str(args.case),normal_offset=args.normal_offset,tangent_offset=args.tangent_offset)):
        result=compare(args.case,args.normal_offset,args.tangent_offset)
    def encode(value):
        if isinstance(value,Plane):return value.to_dict()
        if isinstance(value,np.ndarray):return value.tolist()
        if isinstance(value,np.generic):return value.item()
        raise TypeError(type(value).__name__)
    (args.output/'result.json').write_text(json.dumps(result,default=encode,indent=2))

"""Compare exact base checks and conservative experimental screens, three times."""
import argparse
import hashlib
import json
import os
from pathlib import Path
from time import perf_counter
import numpy as np
from validate_mobile_base_case import load_case, setup
from base_screening import StaticBaseScreen
from motion_toolbox.geometry import as_plane
from motion_toolbox.mobile_base_workflow import generate_base_path
from motion_toolbox.execution import high_qos


@high_qos
def benchmark(case, result_path=None):
    data=load_case(case)['replay']
    targets=[as_plane(t) for t in data['targets']]
    preferred=generate_base_path(targets)['base_planes']
    bases=list(preferred)
    if result_path:
        bases += [as_plane(b) for b in json.loads(result_path.read_text())['base_planes']]
    report=dict(case=str(case),case_sha256=hashlib.sha256((case/'case.json').read_bytes()).hexdigest(),
                poses=len(bases),repetitions=[],production_default='none')
    for trial in range(3):
        _,world=setup(case,data)
        with world:
            baseline=None
            row={}
            # Rotate measurement order to reduce warmup/order bias.
            methods=['none','rectangle','box']
            methods=methods[trial:]+methods[:trial]
            outcomes={}
            for method in methods:
                screen=None if method=='none' else StaticBaseScreen(world,data['environment_meshes'],method)
                checker=world.is_base_valid if screen is None else screen.is_base_valid
                start=perf_counter()
                outcomes[method]=[(checker(b),world.last_failure) for b in bases]
                row[method]=dict(seconds=perf_counter()-start)
                if screen:
                    row[method].update(screen.stats)
                    screen.close()
            for method in ('rectangle','box'):
                row[method]['identical']=outcomes[method]==outcomes['none']
                row[method]['speedup']=row['none']['seconds']/row[method]['seconds']
            report['repetitions'].append(row)
            print(json.dumps(dict(trial=trial,results=row)),flush=True)
    report['screening_gate']={method:all(r[method]['identical'] and r[method]['seconds']<=.9*r['none']['seconds']
        for r in report['repetitions']) for method in ('rectangle','box')}
    report['note']='Default remains none; enabling also requires three matched end-to-end replays without slowdown.'
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',type=Path)
    parser.add_argument('--result',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    os.environ['TOOLBOX_RECORDING']='0'
    report=benchmark(args.case,args.result)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2))

"""Smooth a connected base path, accepting only fully revalidated results."""
import inspect
import math
import numpy as np
from .geometry import Plane, as_plane, rigid_inverse
from .base_planning import BasePlan, plan_mobile_base
from .mobile_transitions import MobileTransitions
from .planning import _in_ranges, normalize_joint_ranges
from .smooth_mobile import moving_average


def smooth_mobile_solution(targets, result, *, windows=(25,10,5,3), **options):
    if not result.configurations or len(targets)<3:
        return result
    solver = options['ik_solver']
    # A refinement solver preserves the connected branch through small edits.
    # Other IK implementations can keep the already validated roadmap result.
    if not hasattr(solver,'refine'):
        return result
    limits = {name:p.default for name,p in inspect.signature(plan_mobile_base).parameters.items()
              if p.default is not inspect.Parameter.empty}
    limits.update(options)
    ranges = normalize_joint_ranges(options.get('joint_ranges'))
    collision = options.get('collision')
    base_valid = options.get('base_valid')
    positions = np.array([b.origin for b in result.base_planes])
    old_roughness = float(np.linalg.norm(np.diff(positions,n=2,axis=0),axis=1).mean())
    trials = []
    for window in windows:
        averaged = moving_average(positions,min(int(window),len(targets)))
        roughness = float(np.linalg.norm(np.diff(averaged,n=2,axis=0),axis=1).mean())
        if roughness>=old_roughness:
            continue
        bases = [Plane(p,b.xaxis,b.yaxis) for p,b in zip(averaged,result.base_planes)]
        configurations = []
        failure = None
        transition = MobileTransitions(limits)
        cost = 0.
        for i,(target,base,seed) in enumerate(zip(targets,bases,result.configurations)):
            target = as_plane(target)
            if base_valid is not None and not base_valid(target,base):
                failure = dict(target=i,reason='placement_or_body')
                break
            T = solver._arm_inverse@rigid_inverse(base.matrix)@target.matrix@solver._tcp_to_flange
            q = solver.refine(T,seed)
            if q is None or not _in_ranges(q,ranges):
                failure = dict(target=i,reason='refinement_or_joint_limits')
                break
            if collision is not None and not collision(q,base):
                checker = getattr(getattr(collision,'func',collision),'__self__',None)
                failure = dict(target=i,reason='configuration_collision',detail=getattr(checker,'last_failure',None))
                break
            previous = ((configurations[i-1],bases[i-1]) if i else
                        (options['current_pose'],as_plane(options['start_base'])) if options.get('current_pose') is not None else None)
            if previous is not None:
                transition.reset()
                if next(transition.reachable(i,[previous],q,base),None) is None:
                    failure = transition.diagnostic(i)
                    break
                oldq,oldbase = previous
                delta = np.asarray(q)-oldq
                periodic = np.asarray(options['periodic'] if options.get('periodic') is not None else [False]*len(q),dtype=bool)
                delta[periodic] = (delta[periodic]+np.pi)%(2*np.pi)-np.pi
                dyaw = (math.atan2(base.xaxis[1],base.xaxis[0])-math.atan2(oldbase.xaxis[1],oldbase.xaxis[0])+np.pi)%(2*np.pi)-np.pi
                weights = np.asarray(options['joint_weights'] if options.get('joint_weights') is not None else [1.]*len(q))
                cost += float(np.linalg.norm(np.r_[(base.origin-oldbase.origin)*limits['base_weight'],
                    dyaw*limits['yaw_weight'],delta*weights]))
            configurations.append(q)
        trials.append(dict(window=window,accepted=failure is None,failure=failure,xy_second_difference=roughness))
        if failure is None:
            diagnostic = dict(mode='validated_smoothing',trials=trials,selected_window=window,
                previous_xy_second_difference=old_roughness,xy_second_difference=roughness,
                validated_targets=len(targets),validated_transitions=len(targets)-1+int(options.get('current_pose') is not None))
            return BasePlan(bases,configurations,cost,[1]*len(targets),result.diagnostics+[diagnostic],
                ik_solutions_per_node=[[q] for q in configurations],
                target_diagnostics=[dict(checked=True,raw_ik=1,within_joint_limits=1,collision_free=1,
                    rejection_reasons={},scope='validated continuation of selected arm branch') for _ in targets])
    result.diagnostics.append(dict(mode='validated_smoothing',trials=trials,selected_window=None,
                                   reason='retained_original_validated_path'))
    return result

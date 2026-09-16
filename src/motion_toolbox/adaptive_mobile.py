"""Adaptive offset/yaw roadmap with full-resolution trajectory validation."""
from itertools import product
from time import perf_counter
import math
import numpy as np
from .geometry import Plane, as_plane
from .base_planning import plan_mobile_base, BasePlan
from .smooth_mobile import moving_average


def plan_adaptive_mobile(targets, arm_in_base, *, height=0., window=50, knot_gap=20,
                         rounds=8, wall_distances=(.6,.8,1.,1.2,1.4),
                         lateral_offsets=None, yaw_offsets=(-.3,0.,.3),
                         **options):
    """Search offset controls and arm branches, then insert failing originals.

    The sparse graph is only a proposal generator. No coarse edge is presented
    as validated. Every returned target and transition passes plan_mobile_base
    with the caller's original constraints, calibrated solver and collisions.
    Previously evaluated knot layers and exact full-resolution states are reused.
    """
    if knot_gap < 1 or int(knot_gap)!=knot_gap or rounds < 1 or int(rounds)!=rounds or window<1 or int(window)!=window:
        raise ValueError('Adaptive knot gap and rounds must be positive integers')
    if lateral_offsets is None:
        failures = []
        for sign in (1.,-1.):
            result = plan_adaptive_mobile(targets,arm_in_base,height=height,window=window,
                knot_gap=knot_gap,rounds=rounds,wall_distances=wall_distances,
                lateral_offsets=[sign*x for x in (.6,.8,1.,1.2)],yaw_offsets=yaw_offsets,**options)
            if result.configurations:
                return result
            failures.extend(result.diagnostics)
        result.diagnostics = failures
        return result
    started = perf_counter()
    progress = options.pop('_adaptive_progress', lambda **data: None)
    seed = options.pop('_adaptive_seed',None)
    targets = [as_plane(t) for t in targets]
    n = len(targets)
    if not n:
        raise ValueError('At least one target required')
    xy = np.array([t.origin[:2] for t in targets])
    normals = np.array([t.zaxis[:2] for t in targets])
    lengths = np.linalg.norm(normals,axis=1)
    if np.any(lengths<1e-8):
        raise ValueError('Adaptive wall planning requires projected TCP normals')
    normals /= lengths[:,None]
    centre = moving_average(xy,min(int(window),n))
    direction = moving_average(normals,min(int(window),n))
    lengths = np.linalg.norm(direction,axis=1)
    if np.any(lengths<1e-8):
        raise ValueError('Opposing averaged wall normals')
    direction /= lengths[:,None]
    tangent = np.column_stack((-direction[:,1],direction[:,0]))
    parameters = np.array(list(product(wall_distances,lateral_offsets,yaw_offsets)),dtype=float)
    if not len(parameters) or not np.isfinite(parameters).all() or np.any(parameters[:,0]<=0):
        raise ValueError('Finite adaptive offsets and positive wall distances required')
    solver = options['ik_solver']
    # Nominal seeds are inexpensive geometric proposals, never final solutions.
    from .kinematics.calibrated import CalibratedURKinematics
    from .kinematics.solver import URKinematics
    proposal_solver = (URKinematics(solver.parameters,solver.tool,solver.arm_in_base)
                       if isinstance(solver,CalibratedURKinematics) else solver)
    ranges = options.get('joint_ranges')
    collision = options.get('collision')
    base_valid = options.get('base_valid')
    mount = as_plane(arm_in_base).origin
    pose_cache = {}
    exact_knots = set()
    counts = {}
    logs = []
    knots = sorted(set(range(0,n,int(knot_gap)))|{n-1})
    reference = None
    active = []
    if seed is not None:
        control = np.asarray(seed['controls'],dtype=float)
        reference = np.column_stack([np.interp(np.arange(n),seed['knots'],control[:,j]) for j in range(3)])
        active = list(seed['failed_targets'])
        knots = sorted(set(seed['knots'])|set(active))
        exact_knots.update(active)
    options = dict(options,_candidate_cache={},_stop_on_unreachable=False)
    from .stationary_region import StationaryRegion
    regions = {id(t):StationaryRegion([t],arm_in_base,max_distance=1.75,base_height=height,projected=True)
               for t in targets}
    def full_base_valid(target,base):
        return regions[id(target)].metrics(base)['geometry_valid'] and (base_valid is None or base_valid(target,base))
    options['base_valid'] = full_base_valid

    def base_at(i,params):
        wall,side,yaw = params
        d,t = direction[i],tangent[i]
        x,y = math.cos(yaw)*d+math.sin(yaw)*t, -math.sin(yaw)*d+math.cos(yaw)*t
        p = centre[i]-wall*d+side*t
        return Plane((*p,height),(*x,0),(*y,0))

    def layer_at(i):
        states = []
        near_failure = reference is None or any(abs(i-j)<=max(window,2*knot_gap) for j in active)
        choices = list(parameters) if near_failure else []
        if reference is not None:
            choices.append(reference[i])
        seen = set()
        for params in choices:
            key = (i,tuple(params),i in exact_knots)
            if key in seen:
                continue
            seen.add(key)
            if key in pose_cache:
                states.extend(pose_cache[key])
                continue
            accepted = []
            pose_cache[key] = accepted
            base = base_at(i,params)
            arm = base.origin+mount[0]*base.xaxis+mount[1]*base.yaxis
            relative = xy[i]-arm[:2]
            if np.linalg.norm(relative)>1.75 or relative@normals[i]<=0:
                continue
            if base_valid is not None and not base_valid(targets[i],base):
                continue
            ik = solver if i in exact_knots else proposal_solver
            for row in ik(targets[i],base):
                q = np.asarray(row,dtype=float)
                # Equivalent angular representatives only in this proposal
                # graph. Final validation expands actual bounded windings.
                q = q.copy()
                for j in getattr(ik,'revolute_joints',()):
                    q[j] = (q[j]+np.pi)%(2*np.pi)-np.pi
                    if ranges and j<len(ranges) and ranges[j] is not None:
                        lo,hi = ranges[j]
                        if lo is not None and hi is not None:
                            shifts = range(math.ceil((lo-q[j])/(2*np.pi)),math.floor((hi-q[j])/(2*np.pi))+1)
                            values = [q[j]+2*np.pi*k for k in shifts]
                            if not values:
                                break
                            q[j] = min(values,key=abs)
                else:
                    if collision is None or collision(q,base):
                        accepted.append(np.r_[params,q])
            states.extend(accepted)
        result = np.asarray(states,dtype=float)
        counts[i] = len(states)
        progress(stage='knot', target=i, states=len(states), elapsed_seconds=perf_counter()-started)
        return result

    last = BasePlan([],[],float('inf'),[None]*n,[],
        target_diagnostics=[dict(checked=False) for _ in targets],ik_solutions_per_node=[[] for _ in targets])
    for iteration in range(int(rounds)):
        parents, path_layers, costs = [], [], None
        blocked = None
        for k,i in enumerate(knots):
            layer = layer_at(i)
            if not len(layer):
                blocked = dict(target=i,reason='empty_roadmap_layer')
                break
            if costs is None:
                costs = np.zeros(len(layer))
                parent = np.full(len(layer),-1)
            else:
                previous = path_layers[-1]
                dp = layer[:,None,:3]-previous[None,:,:3]
                dq = layer[:,None,3:]-previous[None,:,3:]
                for j in getattr(solver,'revolute_joints',()):
                    if not ranges or j>=len(ranges) or ranges[j] is None or (ranges[j][0] is not None and ranges[j][1] is not None and ranges[j][1]-ranges[j][0]>2*np.pi+1e-8):
                        dq[:,:,j] = (dq[:,:,j]+np.pi)%(2*np.pi)-np.pi
                allowed = (abs(dp[:,:,0])<=.401)&(abs(dp[:,:,1])<=.401)&(abs(dp[:,:,2])<=.601)
                allowed &= np.max(abs(dq),axis=2)<2.5
                values = costs[None,:]+np.sum(dp**2,axis=2)+.005*np.sum(dq**2,axis=2)
                values += .00001*(abs(layer[:,1:2])-1)**2
                values[~allowed] = np.inf
                parent = values.argmin(axis=1)
                costs = values[np.arange(len(layer)),parent]
                if not np.isfinite(costs).any():
                    blocked = dict(target=i,reason='disconnected_roadmap',previous_target=knots[k-1])
                    break
            if reference is not None:
                # Retain already feasible working regions unless changing them
                # helps connect the entire path. Without this term, equivalent
                # coarse optima can needlessly replace a good distant prefix.
                costs += np.sum((layer[:,:3]-reference[i])**2,axis=1)
            parents.append(parent)
            path_layers.append(layer)
        if blocked:
            logs.append(dict(iteration=iteration,**blocked))
            break
        chosen = [int(costs.argmin())]
        for k in range(len(path_layers)-1,0,-1):
            chosen.append(int(parents[k][chosen[-1]]))
        chosen.reverse()
        control = np.array([layer[j,:3] for layer,j in zip(path_layers,chosen)])
        dense = np.column_stack([np.interp(np.arange(n),knots,control[:,j]) for j in range(3)])
        reference = dense
        bases = [base_at(i,p) for i,p in enumerate(dense)]
        progress(stage='proposal', iteration=iteration, bases=bases,knots=knots,controls=control.tolist())
        last = plan_mobile_base(targets,[[b] for b in bases],**options)
        bad = [i for i,c in enumerate(last.candidate_counts) if c==0]
        for diagnostic in last.diagnostics:
            if diagnostic.get('reason')=='transition_blocked':
                i = diagnostic['to_target']
                bad.extend([max(0,i-1),i])
        logs.append(dict(iteration=iteration,knots=len(knots),failed_targets=sorted(set(bad)),
            complete=bool(last.configurations),elapsed_seconds=perf_counter()-started))
        progress(stage='validation',**logs[-1])
        if last.configurations:
            break
        new = set(bad)-set(knots)
        for i in bad:
            if i not in exact_knots:
                exact_knots.add(i)
                new.add(i)
        if not new:
            logs.append(dict(reason='roadmap_refinement_exhausted'))
            break
        knots = sorted(set(knots)|new)
        active = bad
    last.diagnostics.append(dict(mode='adaptive_offset_roadmap',iterations=logs,
        coarse_layers=counts,complete=bool(last.configurations),
        scope='bounded proposal search; only full-resolution validated paths are returned',
        total_seconds=perf_counter()-started))
    if last.configurations:
        from .mobile_smoothing import smooth_mobile_solution
        last = smooth_mobile_solution(targets,last,**options)
    return last

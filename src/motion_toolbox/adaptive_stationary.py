"""Experimental bounded base search: cheap probes, local refinement, exact finals.

Search is approximate. Every returned path uses all targets and all requested
TCP rotations with unchanged detailed configuration-collision checking. The
shortest path is exact for each validated base's discrete candidate set.
"""
import math
from time import perf_counter
import numpy as np

from .geometry import Plane, as_plane
from .stationary_region import StationaryRegion
from .planning import candidates, rotation_offsets, _in_ranges
from .base_planning import BasePlan, _validate_stationary_fast, _collision_failure
from .execution import check_cancel, high_qos
from .recording import recorded

ADAPTIVE_STATIONARY_VERSION = 1


def target_probes(targets, count=16):
    """Spatial/orientation extremes plus deterministic farthest-point coverage."""
    planes = [as_plane(t) for t in targets]
    p = np.array([t.origin for t in planes])
    n = np.array([t.zaxis for t in planes])
    span = np.maximum(np.ptp(p, axis=0), .1)
    features = np.column_stack(((p-p.mean(0))/span, n))
    chosen = list(dict.fromkeys([0, len(p)-1]+[int(f(p[:,a])) for a in range(3)
                                            for f in (np.argmin,np.argmax)]))[:count]
    while len(chosen)<min(count,len(p)):
        distance = np.min(np.sum((features[:,None]-features[chosen])**2,axis=2),axis=1)
        distance[chosen]=-1
        chosen.append(int(np.argmax(distance)))
    return chosen


def polygon_grid(polygon, size):
    polygon=np.asarray(polygon,float)
    if len(polygon)<3:return []
    low,high=polygon[:,1].min(),polygon[:,1].max()
    points=[]
    for row in range(size):
        y=low+(row+.5)*(high-low)/size
        xs=[]
        for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
            if min(a[1],b[1])<=y<max(a[1],b[1]):
                xs.append(a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1]))
        if len(xs)>=2:
            points.extend([[min(xs)+(col+.5)*(max(xs)-min(xs))/size,y] for col in range(size)])
    return points


def footprint_clearance(world, targets):
    """Return a conservative XY clearance preference, NOT a collision test.

    World-space AABBs include empty space and ignore vertical separation.
    Query after positioning the base; never use this preference as proof of
    collision or safety. Includes all configured non-arm collision links.
    """
    planes=[as_plane(t) for t in targets]
    normals=np.array([t.zaxis[:2] for t in planes])
    lengths=np.linalg.norm(normals,axis=1)
    use=lengths>1e-9
    if not use.any():return lambda base: None
    normals=normals[use]/lengths[use,None]
    points=np.array([t.origin[:2] for t in planes])[use]
    bounds=np.einsum('ij,ij->i',points,normals)
    links=sorted(world.static_links.intersection(world.collision_links))
    def measure(base):
        world.set_base(base)
        corners=[]
        for link in links:
            low,high=world.p.getAABB(world.robot,link)
            corners.extend([[x,y] for x in (low[0],high[0]) for y in (low[1],high[1])])
        return float(np.min(bounds-np.max(np.asarray(corners)@normals.T,axis=0))) if corners else None
    return measure


class _Evaluation:
    def __init__(self, solver, collision, cancel):
        self.solver,self.collision,self.cancel=solver,collision,cancel
        self.ik_cache,self.collision_cache={},{}
        self.last_failure=None
        self.revolute_joints=getattr(solver,'revolute_joints',None)
        self.stats=dict(ik_calls=0,ik_cache_hits=0,collision_checks=0,collision_cache_hits=0,
                        ik_seconds=0.,collision_seconds=0.)

    @staticmethod
    def key(base):return as_plane(base).matrix.tobytes()

    def __call__(self,target,base):
        check_cancel(self.cancel)
        key=self.key(base),as_plane(target).matrix.tobytes()
        if key in self.ik_cache:
            self.stats['ik_cache_hits']+=1
        else:
            tick=perf_counter()
            self.ik_cache[key]=list(self.solver(target,base))
            self.stats['ik_calls']+=1
            self.stats['ik_seconds']+=perf_counter()-tick
        return self.ik_cache[key]

    def valid(self,q,base):
        check_cancel(self.cancel)
        key=self.key(base),tuple(q)
        if key in self.collision_cache:
            self.stats['collision_cache_hits']+=1
        else:
            tick=perf_counter()
            valid=True if self.collision is None else self.collision(q,base)
            reason=_collision_failure(self.collision)
            self.collision_cache[key]=(valid,reason)
            self.stats['collision_checks']+=int(self.collision is not None)
            self.stats['collision_seconds']+=perf_counter()-tick
        valid,self.last_failure=self.collision_cache[key]
        return valid


@recorded
@high_qos
def find_adaptive_stationary_base(targets, *, ik_solver, arm_in_base,
        collision=None, base_collision=None, candidate_planes=(), current_pose=None,
        joint_ranges=None, periodic=None, rotation_steps=24, max_joint_step=2.5,
        step_limits=None, build_path=True, base_height=0., grid_size=5, yaw_steps=8,
        probe_count=16, probe_rotations=8, beam_width=4, refinement_steps=(.16,.08),
        validation_probe_count=64, connected_finalists=1,
        max_full_checks=3, initial_reach=1.75, exploration_reach=2.1,
        tool_clearance=1., clearance_weight=.15, clearance_measure=None,
        heading_bias=1., cancel_check=None, progress=None):
    """Return {found, search_diagnostics, stats, ...}; caches last one call only.

    Full validations rank connected results by exact joint-path cost. Probe
    robustness/clearance only ranks search proposals, never validates safety.
    No all-target solution counts are inferred from sparse probes.
    """
    started=perf_counter()
    targets=[as_plane(t) for t in targets]
    if not targets:raise ValueError('At least one target required')
    for name,value,minimum in [('grid_size',grid_size,2),('yaw_steps',yaw_steps,1),('probe_count',probe_count,1),
            ('probe_rotations',probe_rotations,1),('beam_width',beam_width,1),
            ('validation_probe_count',validation_probe_count,1),
            ('connected_finalists',connected_finalists,1),
            ('max_full_checks',max_full_checks,1),('rotation_steps',rotation_steps,1)]:
        if int(value)!=value or value<minimum:raise ValueError(name+' must be a positive integer')
    (grid_size,yaw_steps,probe_count,probe_rotations,beam_width,max_full_checks,
     rotation_steps,validation_probe_count,connected_finalists)=map(int,(
        grid_size,yaw_steps,probe_count,probe_rotations,beam_width,max_full_checks,
        rotation_steps,validation_probe_count,connected_finalists))
    if any(not math.isfinite(v) or v<=0 for v in (initial_reach,exploration_reach,*refinement_steps)):
        raise ValueError('Reach and refinement distances must be positive finite metres')
    if exploration_reach<initial_reach or any(not math.isfinite(v) or v<0 for v in (tool_clearance,clearance_weight,heading_bias)):
        raise ValueError('Invalid reach, clearance or heading preference')
    if current_pose is not None and (not np.isfinite(current_pose).all() or not _in_ranges(current_pose,joint_ranges)):
        raise ValueError('Starting configuration exceeds joint limits or is nonfinite')
    region=StationaryRegion(targets,arm_in_base,max_distance=initial_reach,base_height=base_height,projected=True)
    initial,_=region.polygon(cancel_check=cancel_check)
    outer=StationaryRegion(targets,arm_in_base,max_distance=exploration_reach,base_height=base_height,projected=True)
    probes=target_probes(targets,min(probe_count,len(targets)))
    validation_probes=target_probes(targets,min(validation_probe_count,len(targets)))
    # Low TCPs are close to chassis/lift geometry in wall tasks. Ordering changes
    # only discovery latency; every accepted finalist still checks every target.
    validation_probes.sort(key=lambda i:targets[i].origin[2])
    offsets=rotation_offsets('n_steps',steps=rotation_steps)
    sample_offsets=[offsets[i] for i in sorted({int(i*rotation_steps/min(probe_rotations,rotation_steps))
                                               for i in range(min(probe_rotations,rotation_steps))})]
    evaluation=_Evaluation(ik_solver,collision,cancel_check)
    records={}
    body_checks=0
    def report(stage,**data):
        check_cancel(cancel_check)
        if progress:progress(dict(stage=stage,elapsed_seconds=perf_counter()-started,**data))
    def make(point,yaw):
        c,s=math.cos(yaw),math.sin(yaw);x,y=region.mount.origin[:2]
        return Plane((point[0]-c*x+s*y,point[1]-s*x-c*y,base_height),(c,s,0),(-s,c,0))
    def proposal_key(base):return np.round(base.matrix,10).tobytes()
    def add_pose(base,stage,relative):
        records.setdefault(proposal_key(base),dict(base=base,stage=stage,
            side_heading=(1-math.cos(2*relative))/2,probe_done=False))
    def add(points,stage):
        for point in points:
            toward=region.center-np.asarray(point)
            facing=math.atan2(toward[1],toward[0])-math.atan2(region.mount.xaxis[1],region.mount.xaxis[0])
            # Study prior, not a heading exclusion: cover the complete circle.
            for relative in -math.pi/2+np.arange(yaw_steps)*2*math.pi/yaw_steps:
                add_pose(make(point,facing+relative),stage,float(relative))
    supplied=[as_plane(b) for b in candidate_planes]
    if supplied:
        for b in supplied:records.setdefault(proposal_key(b),dict(base=b,stage='supplied',side_heading=False,probe_done=False))
    else:
        add(polygon_grid(initial+region.center,grid_size),'coarse')
        _,seeds,_=region.candidates(spacing=.5,cancel_check=cancel_check)
        add([region.metrics(b)['arm_origin'][:2] for b in seeds[::4]],'seed')
        polygon,_=outer.polygon(cancel_check=cancel_check)
        add(polygon_grid(polygon+outer.center,3),'outer')
    def probe(row):
        nonlocal body_checks
        check_cancel(cancel_check)
        if row['probe_done']:return
        row.update(probe_done=True,probe_targets=[],probe_reached=0,probe_min_ik=0,
                   probe_mean_ik=0.,score=0.,rejected=None,clearance=None)
        base=row['base'];body_checks+=int(base_collision is not None)
        if base_collision is not None and not base_collision(base):
            row['rejected']='base_collision';return
        if current_pose is not None and not evaluation.valid(current_pose,base):
            row['rejected']='initial_collision'
            row['initial_state_failure']=evaluation.last_failure or 'collision checker rejected starting configuration'
            return
        if clearance_measure is not None:row['clearance']=clearance_measure(base)
        metrics=region.metrics(base);arm=np.asarray(metrics['arm_origin'])
        order=sorted(probes,key=lambda i:-np.linalg.norm(targets[i].origin-arm))
        counts=[];free_estimates=[];valid_checks=0;failures=[]
        for i in order:
            qs,_,_=candidates(targets[i],base,evaluation,sample_offsets,None,joint_ranges)
            row['probe_targets'].append(i);counts.append(len(qs))
            if not qs:
                free_estimates.append(0.)
                failures.append(dict(target=i,reason='probe_ik'));continue
            # A bounded witness search is a heuristic, not an exact alternative count.
            indices=list(dict.fromkeys(np.linspace(0,len(qs)-1,min(8,len(qs)),dtype=int).tolist()))
            valid=0
            for j in indices:
                valid_checks+=1
                if evaluation.valid(qs[j],base):valid+=1
            estimate=len(qs)*valid/len(indices)
            if not valid:
                # Finish the sampled layer before calling it unreachable; a small
                # subset of collision checks must not hide a viable branch.
                for j in range(len(qs)):
                    if j in indices:continue
                    valid_checks+=1
                    if evaluation.valid(qs[j],base):valid=1;estimate=1.;break
            if not valid:
                failures.append(dict(target=i,reason='probe_collision',detail=evaluation.last_failure))
            else:row['probe_reached']+=1
            free_estimates.append(estimate)
        row['probe_min_ik']=min(counts) if counts else 0
        row['probe_mean_ik']=float(np.mean(counts)) if counts else 0.
        row['probe_collision_checks']=valid_checks
        row['probe_estimated_min_free']=min(free_estimates) if free_estimates else 0.
        row['probe_estimated_mean_free']=float(np.mean(free_estimates)) if free_estimates else 0.
        row['probe_failures']=failures
        row['rejected']=failures[0]['reason'] if failures else None
        if counts:
            health=math.sqrt((1+row['probe_estimated_min_free'])*(1+row['probe_estimated_mean_free']))
            clearance=0. if row['clearance'] is None or tool_clearance==0 else np.clip(row['clearance']/tool_clearance,-1.,1.)
            row['score']=float(health*(1+heading_bias*row['side_heading'])*(1+clearance_weight*clearance))
    def ranking():
        return sorted((r for r in records.values() if r['rejected'] not in ('base_collision','initial_collision')),
                      key=lambda r:(-r['probe_reached'],-r['score'],evaluation.key(r['base'])))
    def diverse_ranking():
        # Reserve candidates from distinct heading families at equal coverage.
        # High raw IK counts must not crowd out the side-on branch observed in
        # the study. Remaining positions retain their deterministic ranking.
        ranked=ranking()
        if not ranked:return []
        leaders=[];rest=[];families=set()
        for row in ranked:
            base=row['base'];xy=np.array(region.metrics(base)['arm_origin'][:2])
            toward=region.center-xy
            relative=math.atan2(base.xaxis[1],base.xaxis[0])-math.atan2(toward[1],toward[0])
            relative+=math.atan2(region.mount.xaxis[1],region.mount.xaxis[0])
            family=int(math.floor(relative/(math.pi/2)+.5))%4
            if row['probe_reached']==ranked[0]['probe_reached'] and family not in families:
                leaders.append(row);families.add(family)
            else:rest.append(row)
        return leaders+rest
    for level,step in enumerate([None]+([] if supplied else list(refinement_steps))):
        if step is not None:
            # Refine position AND heading. The hose changes which heading is
            # viable, so a quarter-turn prior must never freeze the orientation.
            parents=[]
            for row in diverse_ranking():
                xy=np.array(region.metrics(row['base'])['arm_origin'][:2])
                yaw=math.atan2(row['base'].xaxis[1],row['base'].xaxis[0])
                if all(np.linalg.norm(xy-p)>.75*step or abs(math.atan2(math.sin(yaw-a),math.cos(yaw-a)))>.4
                       for p,a in parents):parents.append((xy,yaw))
                if len(parents)==beam_width:break
            for point,yaw in parents:
                for dx,dy in ((-1,0),(1,0),(0,-1),(0,1),(0,0)):
                    xy=point+np.array([dx,dy])*step
                    toward=region.center-xy
                    facing=math.atan2(toward[1],toward[0])-math.atan2(region.mount.xaxis[1],region.mount.xaxis[0])
                    for turn in ((-math.pi/8,0.,math.pi/8) if dx==dy==0 else (0.,)):
                        add_pose(make(xy,yaw+turn),'refine_'+str(level),yaw+turn-facing)
        pending=[r for r in records.values() if not r['probe_done']]
        for i,row in enumerate(pending):
            probe(row)
            if i%10==0:report('adaptive_probes',level=level,done=i+1,total=len(pending),candidates=len(records))
    best=None;failures=[];validated=[];critical=[];critical_targets=[]
    graph_seconds=0.;bottleneck_checks=0
    # Failure-driven repair: a failed adjacent transition becomes a cheap exact
    # two-layer test for subsequent bases. No hardcoded target indices.
    def bottleneck(row):
        nonlocal graph_seconds,bottleneck_checks
        if not critical and not critical_targets:return True
        bottleneck_checks+=1
        if bottleneck_checks%10==1:report('adaptive_bottlenecks',checked=bottleneck_checks)
        from .graph import shortest_path
        base=row['base']
        for index in critical_targets:
            qs,_,_=candidates(targets[index],base,evaluation,offsets,None,joint_ranges)
            if not any(evaluation.valid(q,base) for q in qs):
                row['failed_priority_target']=index
                return False
        for left,right in critical:
            a,_,_=candidates(targets[left],base,evaluation,offsets,None,joint_ranges)
            b,_,_=candidates(targets[right],base,evaluation,offsets,None,joint_ranges)
            checked={}
            def valid(i,j):
                key=i,j
                if key not in checked:checked[key]=evaluation.valid((a,b)[i][j],base)
                return checked[key]
            limits=max_joint_step if step_limits is None else np.minimum(max_joint_step,step_limits[right])
            graph_stats={}
            result=shortest_path([a,b],max_step=limits,periodic=periodic,count_paths=False,
                node_valid=valid,revolute_joints=evaluation.revolute_joints,
                node_rejection_group=lambda i,j:[k for k in range(len((a,b)[i])) if not valid(i,k)],
                cancel_check=lambda:check_cancel(cancel_check),stats=graph_stats)
            graph_seconds+=graph_stats.get('graph_seconds',0.)
            if not result.configurations:return False
        return True
    def screen_finalist(row):
        # A denser reachability screen avoids building all 1,102 layers just to
        # discover an interior target missed by the 16 coarse representatives.
        # A failed sampled layer is completed at ALL requested rotations before
        # rejecting the base. Successful witnesses are sufficient for screening.
        base=row['base']
        for index in validation_probes:
            qs,_,_=candidates(targets[index],base,evaluation,sample_offsets,None,joint_ranges)
            if any(evaluation.valid(q,base) for q in qs):continue
            qs,_,_=candidates(targets[index],base,evaluation,offsets,None,joint_ranges)
            if any(evaluation.valid(q,base) for q in qs):continue
            row['failed_priority_target']=index
            if index not in critical_targets:critical_targets.append(index)
            failure=_validate_stationary_fast(targets,base,current_pose,evaluation,
                evaluation.valid if collision is not None else None,region.metrics(base),False,
                dict(rotation_mode='n_steps',rotation_steps=rotation_steps,joint_ranges=joint_ranges,
                     _priority_targets=[index],cancel_check=cancel_check))
            failures.append(failure)
            return False
        return True
    attempts=0;full_checks=0;connected_count=0
    for row in diverse_ranking():
        if full_checks>=max_full_checks:break
        if not bottleneck(row):row['bottleneck_rejected']=True;continue
        if not screen_finalist(row):row['reachability_screen_rejected']=True;continue
        attempts+=1;report('adaptive_full_validation',attempt=attempts,score=row['score'])
        options=dict(rotation_mode='n_steps',rotation_steps=rotation_steps,joint_ranges=joint_ranges,
            periodic=periodic,max_joint_step=max_joint_step,step_limits=step_limits,count_paths=False,
            _max_lazy_passes=2,
            cancel_check=cancel_check,progress=progress,_priority_targets=list(dict.fromkeys(
                critical_targets+[i for pair in critical for i in pair]+validation_probes+
                sorted(range(len(targets)),key=lambda i:targets[i].origin[2]))))
        result=_validate_stationary_fast(targets,row['base'],current_pose,evaluation,
            evaluation.valid if collision is not None else None,region.metrics(row['base']),build_path,options)
        graph_seconds+=sum(d.get('timings',{}).get('path_seconds',0.) for d in result.diagnostics)
        complete=len(result.configurations)==len(targets) if build_path else bool(result.base_planes)
        # A base rejected before proving all-target reachability is not a fully
        # evaluated finalist. Learn its blocker without spending the finalist
        # budget, so a run of similar bad poses cannot hide a viable side pose.
        full_checks+=int(bool(result.base_planes))
        validated.append(dict(base=row['base'].to_dict(),connected=bool(build_path and complete),reachable=bool(result.base_planes),
            cost=result.cost if math.isfinite(result.cost) else None,disconnected_detail=result.disconnected_detail,
            probe_score=row['score'],failure_reason=result.diagnostics[-1].get('reason') if result.diagnostics else None,
            failed_targets=result.diagnostics[-1].get('unreachable_points',[]) if result.diagnostics else [],
            failure_detail=result.diagnostics[-1].get('failed_target_details') if result.diagnostics else None))
        if complete:
            connected_count+=1
            if best is None or (build_path and result.cost<best.cost):best=result
            if connected_count>=connected_finalists:break
        else:
            failures.append(result)
            for index in validated[-1]['failed_targets']:
                if index not in critical_targets:critical_targets.append(index)
            detail=result.disconnected_detail
            if detail and detail['from_target']>=0:
                pair=(detail['from_target'],detail['to_target'])
                if pair not in critical:critical.append(pair)
    if best is None:
        best=next((r for r in reversed(failures) if r.base_planes),None)
        if best is None:
            best=failures[-1] if failures else BasePlan([],[],float('inf'),[],[],counts_complete=False,ik_option_count=None)
    best.validation_attempts=attempts;best.base_collision_checks=body_checks
    if not failures and not best.base_planes:
        best.initial_state_failure=next((r['initial_state_failure'] for r in records.values()
                                        if r.get('initial_state_failure')),None)
    best.heuristic_plane=ranking()[0]['base'] if ranking() else None
    diagnostics=[dict(base=r['base'].to_dict(),**{k:v for k,v in r.items() if k not in ('base','probe_done')})
                 for r in records.values()]
    complete=bool(best.configurations) if build_path else bool(best.base_planes)
    report('adaptive_complete',connected=complete,full_checks=full_checks,candidates=len(records))
    return dict(found=best,search_diagnostics=diagnostics,validated_candidates=validated,
        candidate_count=len(records),probe_indices=probes,learned_bottlenecks=critical,
        validation_probe_indices=validation_probes,
        learned_failed_targets=critical_targets,
        counts_complete=False,global_optimum_proven=False,
        objective='lowest joint-path cost among fully validated connected candidates',
        status=('connected' if build_path else 'reachable_without_path') if complete else 'search_budget_exhausted_without_connected_path',
        stats=dict(evaluation.stats,base_collision_checks=body_checks,full_checks=full_checks,validation_attempts=attempts,
                   bottleneck_checks=bottleneck_checks,graph_seconds=graph_seconds,
                   total_seconds=perf_counter()-started))

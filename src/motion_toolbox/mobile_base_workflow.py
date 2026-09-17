"""Smooth mobile-base proposal and full ordered IK/collision validation.

No base search or automatic constraint relaxation. Shared APIs have no timeout;
callers may provide a cooperative cancel_check callback.
"""
from collections import Counter
from contextlib import ExitStack
from functools import partial
from time import perf_counter
import math
import numpy as np
from .recording import recorded
from .geometry import Plane, as_plane
from .xy_smoothing import smooth_xy
from .xy_centerline import centerline_xy
from .xy_offset import centerline_offset_frames
from .stationary_region import StationaryRegion
from .planning import candidates, rotation_offsets
from .graph import shortest_path


def generate_base_path(targets, *, max_xy_deviation=.25, normal_offset=.9, tangent_offset=1.2):
    targets = [as_plane(t) for t in targets]
    smoothing = smooth_xy([t.origin for t in targets], max_xy_deviation)
    guide = centerline_xy(smoothing['curve'])
    frames = centerline_offset_frames(guide['curve'], guide['mapped_points'],
        [t.xaxis for t in targets], [t.yaxis for t in targets],
        x_offset=-normal_offset, y_offset=tangent_offset, pass_points=smoothing['curve'])
    bases = [Plane(o,x,y) for o,x,y in zip(frames['origins'],frames['x_axes'],frames['y_axes'])]
    return dict(base_planes=bases, smoothing=smoothing, centerline=guide,
                target_indices=list(range(len(targets))))


@recorded
def validate_base_path(targets, bases, *, solver, world, joint_ranges, periodic,
                       current_pose=None, rotation_steps=16, max_joint_step=2.5, max_base_step=.25,
                       max_yaw_step=.25, max_reach_xy=1.75, collision_options=None,
                       time_intervals=None, max_base_speed=None, max_yaw_speed=None,
                       max_joint_speed=None, cancel_check=None, progress=None):
    """Check every target, then connect feasible states with sampled swept checks.

    time_intervals contains one duration per transition (N-1, or N with a start
    configuration). The optional starting arm configuration is at bases[0]; an
    approach from another base location is not planned. Speeds require times.
    """
    started=perf_counter()
    offsets=rotation_offsets("n_steps",steps=rotation_steps)
    targets,bases=[as_plane(t) for t in targets],[as_plane(b) for b in bases]
    if not targets or len(targets)!=len(bases):
        raise ValueError('Require exactly one base plane per target')
    if world is None:
        raise ValueError('A configured collision world is required')
    for name,value in [('max_base_step',max_base_step),('max_yaw_step',max_yaw_step),('max_reach_xy',max_reach_xy)]:
        if not math.isfinite(value) or value<=0: raise ValueError(name+' must be finite and positive')
    step=np.broadcast_to(np.asarray(max_joint_step,dtype=float),(6,))
    if not np.isfinite(step).all() or np.any(step<=0): raise ValueError('max_joint_step must be finite and positive')
    periodic=np.asarray(periodic,dtype=bool)
    options=dict(collision_options or {})
    clearance=options.get('clearance',0.)
    edge_options={k:options[k] for k in ('joint_resolution','base_resolution','yaw_resolution') if k in options}
    durations=None if time_intervals is None else np.asarray(time_intervals,dtype=float)
    expected=len(targets)-int(current_pose is None)
    if durations is not None and (durations.shape!=(expected,) or not np.isfinite(durations).all() or np.any(durations<=0)):
        raise ValueError('time_intervals must contain {} finite positive durations'.format(expected))
    if any(v is not None for v in (max_base_speed,max_yaw_speed,max_joint_speed)) and durations is None:
        raise ValueError('Speed limits require time_intervals')
    for limit in (max_base_speed,max_yaw_speed,max_joint_speed):
        if limit is not None and (not np.isfinite(limit).all() or np.any(np.asarray(limit)<=0)):
            raise ValueError('Speed limits must be finite and positive')
    def check():
        if cancel_check is not None: cancel_check()
    transitions=[]
    for i in range(1,len(bases)):
        d=float(np.linalg.norm(bases[i].origin-bases[i-1].origin))
        yaw=[math.atan2(b.xaxis[1],b.xaxis[0]) for b in bases[i-1:i+1]]
        angle=abs((yaw[1]-yaw[0]+math.pi)%(2*math.pi)-math.pi)
        errors=[]
        for name,value,limit in [('translation',d,max_base_step),('yaw',angle,max_yaw_step)]:
            if value>limit+1e-12: errors.append(dict(reason=name,measured=value,limit=limit))
        if durations is not None:
            dt=durations[i-1 if current_pose is None else i]
            for name,value,limit in [('base_speed',d/dt,max_base_speed),('yaw_speed',angle/dt,max_yaw_speed)]:
                if limit is not None and value>limit+1e-12: errors.append(dict(reason=name,measured=value,limit=limit))
        if errors: transitions.append(dict(from_target=i-1,to_target=i,rejections=errors))
    layers,diagnostics,layer_angles=[],[],[]
    def joint_key(q):
        values=np.asarray(q,dtype=float).copy()
        angular=list(getattr(solver,'revolute_joints',()))
        values[angular]=(values[angular]+np.pi)%(2*np.pi)-np.pi
        return tuple(np.round(values,8))
    for i,(target,base) in enumerate(zip(targets,bases)):
        check()
        angles={}
        placement=StationaryRegion([target],solver.arm_in_base,max_distance=max_reach_xy,projected=True).metrics(base)
        detail=dict(index=i,placement=placement,raw_ik=None,within_joint_limits=None,collision_free=None,rejection_reasons={})
        if not np.allclose(base.zaxis,[0,0,1],atol=1e-10) or abs(base.origin[2])>1e-10:
            detail.update(state='placement_rejection',reason='Base is not upright at ground Z=0')
            rows=[]
        elif not placement['geometry_valid']:
            detail.update(state='placement_rejection',reason='Negative target-Z side or calibrated arm-origin XY reach rule',reach_limit=max_reach_xy)
            rows=[]
        elif not world.is_base_valid(base,clearance=clearance):
            detail.update(state='body_collision',reason=world.last_failure)
            rows=[]
        else:
            def tracked_solver(rotated, footprint):
                check()
                angle=math.atan2(np.dot(target.yaxis,rotated.xaxis),np.dot(target.xaxis,rotated.xaxis))%(2*math.pi)
                solutions=solver(rotated,footprint)
                for q in solutions: angles.setdefault(joint_key(q),angle)
                return solutions
            tracked_solver.revolute_joints=getattr(solver,'revolute_joints',())
            rows,_,_=candidates(target,base,tracked_solver,offsets,partial(world.is_valid,clearance=clearance),joint_ranges,stats=detail)
            detail['state']=('feasible_state' if rows else 'no_ik' if not detail['raw_ik'] else
                             'joint_limit_rejection' if not detail['within_joint_limits'] else 'configuration_collision')
        detail["rotation_steps"]=int(rotation_steps)
        diagnostics.append(detail);layers.append(rows)
        layer_angles.append([angles[joint_key(q)] for q in rows])
        if progress is not None and ((i+1)%100==0 or i+1==len(targets)):
            progress(dict(stage='target_validation',tested=i+1,total=len(targets),
                state_counts=dict(Counter(d['state'] for d in diagnostics)),
                elapsed_seconds=perf_counter()-started))
    unreachable=[i for i,rows in enumerate(layers) if not rows]
    edge_failures={}
    edge_cache={}
    edge_cache_layer=[None]
    edge_cache_hits=[0]
    physical_key=getattr(world,'configuration_cache_key',None)
    def edge_valid(i,a,b):
        check()
        q0=current_pose if i==0 else layers[i-1][a]
        base0=bases[0] if i==0 else bases[i-1]
        q1=layers[i][b]
        if max_joint_speed is not None:
            delta=np.asarray(q1)-q0
            delta[periodic]=(delta[periodic]+math.pi)%(2*math.pi)-math.pi
            dt=durations[i-1 if current_pose is None else i]
            speed=abs(delta)/dt
            if np.any(speed>np.asarray(max_joint_speed)+1e-12):
                edge_failures.setdefault(i,Counter())['joint speed limit: measured {} rad/s, limit {}'.format(speed.tolist(),max_joint_speed)]+=1
                return False
        if edge_cache_layer[0]!=i:
            edge_cache.clear();edge_cache_layer[0]=i
            if progress is not None and i%100==0:
                progress(dict(stage='joint_graph',target=i,total=len(targets),
                    edge_cache_hits=edge_cache_hits[0],elapsed_seconds=perf_counter()-started))
        cache_key=None
        if physical_key is not None:
            k0,k1=physical_key(q0),physical_key(q1)
            if k0[0]!='out_of_range' and k1[0]!='out_of_range':
                delta=np.asarray(q1)-q0
                delta[periodic]=(delta[periodic]+math.pi)%(2*math.pi)-math.pi
                # Preserve the exact number of sampled states: tiny roundoff at
                # a resolution boundary must not change collision coverage.
                y0=math.atan2(base0.xaxis[1],base0.xaxis[0])
                y1=math.atan2(bases[i].xaxis[1],bases[i].xaxis[0])
                yaw=abs((y1-y0+math.pi)%(2*math.pi)-math.pi)
                samples=max(1,math.ceil(np.max(abs(delta))/edge_options.get('joint_resolution',.05)),
                    math.ceil(np.linalg.norm(bases[i].origin-base0.origin)/edge_options.get('base_resolution',.02)),
                    math.ceil(yaw/edge_options.get('yaw_resolution',.05)))
                cache_key=(k0,k1,tuple(np.round(delta,10)),samples)
        if cache_key is not None and cache_key in edge_cache:
            accepted,reason=edge_cache[cache_key];edge_cache_hits[0]+=1
        else:
            accepted=world.edge_is_valid(q0,base0,q1,bases[i],periodic=periodic,clearance=clearance,**edge_options)
            reason=world.last_failure or 'sampled swept collision'
            if cache_key is not None: edge_cache[cache_key]=(accepted,reason)
        if not accepted: edge_failures.setdefault(i,Counter())[reason]+=1
        return accepted
    solved=None
    if not unreachable and not transitions:
        if progress is not None: progress(dict(stage='joint_graph',target=0,total=len(targets),state_counts=dict(Counter(d['state'] for d in diagnostics))))
        solved=shortest_path(layers,start=current_pose,periodic=periodic,max_step=max_joint_step,
            edge_valid=edge_valid,count_paths=False,revolute_joints=getattr(solver,'revolute_joints',None))
    configurations=solved.configurations if solved is not None else []
    valid=len(configurations)==len(targets)
    selected_angles=[layer_angles[i][j] for i,j in enumerate(solved.indices)] if valid else []
    selected_targets=[target.rotated_z(angle) for target,angle in zip(targets,selected_angles)]
    failure_layer=solved.failure_layer if solved is not None else None
    disconnected_detail=None
    if failure_layer is not None:
        previous=([current_pose] if failure_layer==0 else
                  [layers[failure_layer-1][i] for i in solved.reachable_indices])
        best=None
        limit=np.broadcast_to(np.asarray(max_joint_step,dtype=float),(6,))
        for q0 in previous:
            delta=np.asarray(layers[failure_layer])-q0
            delta[:,periodic]=(delta[:,periodic]+math.pi)%(2*math.pi)-math.pi
            score=np.max(abs(delta)/limit,axis=1)
            j=int(np.argmin(score))
            if best is None or score[j]<best[0]: best=(float(score[j]),abs(delta[j]).tolist())
        disconnected_detail=dict(from_target=failure_layer-1,to_target=failure_layer,
            minimum_joint_step_limit_ratio=None if best is None else best[0],
            joint_deltas_at_nearest_pair=None if best is None else best[1],joint_step_limits=limit.tolist(),
            edge_rejections=dict(edge_failures.get(failure_layer,{})))
    counts=dict(Counter(d['state'] for d in diagnostics))
    if valid: status='Validated {} targets and a connected arm path with sampled swept collisions.'.format(len(targets))
    elif unreachable:
        i=unreachable[0];status='Proposal failed at target {}: {}. No validated arm path.'.format(i,diagnostics[i]['state'])
    elif transitions: status='Proposal failed at base transition {} -> {}.'.format(transitions[0]['from_target'],transitions[0]['to_target'])
    else: status='Disconnected arm path at transition {} -> {} (joint-step or sampled swept constraints).'.format(failure_layer-1 if failure_layer is not None else '?',failure_layer)
    return dict(base_planes=bases,configurations=configurations,fabrication_validated=valid,
        edge_cache_hits=edge_cache_hits[0],rotation_steps=int(rotation_steps),selected_tcp_rotations=selected_angles,selected_target_planes=selected_targets,
        status=status,target_diagnostics=diagnostics,state_counts=counts,unreachable_points=unreachable,
        unchecked_points=[],transition_failures=transitions,disconnected_target=failure_layer,
        disconnected_detail=disconnected_detail,edge_rejection_reasons={str(k):dict(v) for k,v in edge_failures.items()},
        max_joint_step=max_joint_step,path_length=solved.cost if solved else None,
        collision_check_applied=True,check_edges=True,speed_checked=any(v is not None for v in (max_base_speed,max_yaw_speed,max_joint_speed)),
        validation_seconds=perf_counter()-started)


@recorded
def plan_mobile_base(robot, target_planes, *, units_to_metres=1., max_xy_deviation=.25,
                     normal_offset=.9, tangent_offset=1.2, current_pose=None, arm_in_base=None,
                     arm_joint_names=None, fixed_joint_values=None, joint_ranges=None,
                     collision_meshes=(), collision_options=None, group=None, parameters=None,
                     scene=None, cancel_check=None, **limits):
    """Robot boundary: inputs planes/meshes in model units; geometric settings metres.

    Robot/tool and fixed joints are metres/radians. Targets may rotate about local
    TCP Z (16 samples by default), preserving positions and extrusion directions. Collision and sampled edge validation are mandatory.
    """
    from .robot_adapter import kinematics_from_robot,resolve_arm_joint_names,configuration_from_values,_active_tool
    from .robot_planning import json_input
    from .collision import PybulletServer
    if robot is None: raise ValueError('Connect robot with its calibrated active tool')
    if not math.isfinite(units_to_metres) or units_to_metres<=0: raise ValueError('units_to_metres must be positive')
    if _active_tool(robot,group) is None: raise ValueError('Attach the calibrated active tool before mobile validation')
    targets=[as_plane(t,units_to_metres) for t in target_planes]
    proposal=generate_base_path(targets,max_xy_deviation=max_xy_deviation,normal_offset=normal_offset,tangent_offset=tangent_offset)
    if cancel_check: cancel_check()
    names=resolve_arm_joint_names(robot,arm_joint_names,group=group)
    joints={j.name:j for j in robot.model.get_configurable_joints()}
    fixed=dict(json_input(fixed_joint_values,{}))
    if hasattr(current_pose,'joint_values'):
        values=dict(zip(current_pose.joint_names,current_pose.joint_values))
        if any(n not in values for n in names): raise ValueError('Starting Configuration is missing arm joints')
        fixed={**{n:v for n,v in values.items() if n not in names},**fixed}
        start=[values[n] for n in names]
    else: start=list(current_pose) if current_pose is not None and len(current_pose) else None
    if start is not None and (len(start)!=6 or not np.isfinite(start).all()): raise ValueError('current_pose requires six finite radians')
    if any(n not in joints or n in names or not math.isfinite(v) for n,v in fixed.items()): raise ValueError('Invalid fixed joint values')
    solver=kinematics_from_robot(robot,parameters=json_input(parameters),group=group,arm_joint_names=names,
        fixed_joint_values=fixed,arm_in_base=as_plane(arm_in_base,units_to_metres) if arm_in_base is not None else None)
    fixed={**{n:0. for n in joints if n not in names},**fixed,**solver.fixed_joint_values}
    ranges=json_input(joint_ranges)
    if ranges is None: ranges=[([joints[n].limit.lower,joints[n].limit.upper] if joints[n].limit is not None and joints[n].type!=1 else None) for n in names]
    options=dict(json_input(collision_options,{}))
    allowed={'gui','allowed_pairs','package_paths','asset_root','ground_z','support_links','joint_resolution','base_resolution','yaw_resolution','clearance','check_static_self_collisions'}
    if set(options)-allowed: raise ValueError('Unknown collision options: '+str(set(options)-allowed))
    with ExitStack() as stack:
        if scene is not None:
            if collision_meshes or options: raise ValueError('External scene must already contain environment; omit collision_meshes/options')
            world=scene
            if list(world.joint_names)!=names: raise ValueError('Collision scene joint order differs')
        else:
            constructor={k:options[k] for k in ('gui','allowed_pairs','package_paths','asset_root','check_static_self_collisions') if k in options}
            constructor.setdefault('check_static_self_collisions',False)
            world=stack.enter_context(PybulletServer(robot=robot,joint_names=names,**constructor))
            for mesh in collision_meshes: world.add_mesh(mesh,scale=units_to_metres)
            if 'ground_z' in options: world.add_ground(options['ground_z'],support_links=options.get('support_links',()))
        world.set_fixed_joints(fixed)
        validation=validate_base_path(targets,proposal['base_planes'],solver=solver,world=world,joint_ranges=ranges,
            periodic=[joints[n].type==1 for n in names],current_pose=start,collision_options=options,cancel_check=cancel_check,**limits)
    proposal.update(validation)
    output_names=[n for n,j in joints.items() if j.type==2 and n not in names]+names
    proposal['configuration_objects']=[configuration_from_values([dict(fixed,**dict(zip(names,q)))[n] for n in output_names],output_names,[joints[n].type for n in output_names]) for q in validation['configurations']]
    proposal.update(arm_in_base=solver.arm_in_base,fixed_joint_values=fixed,mounting_source=solver.mounting_source)
    proposal['effective_settings']=dict(units_to_metres=units_to_metres,
        rotation_steps=validation['rotation_steps'],solver=type(solver).__name__,
        mounting_source=solver.mounting_source,arm_in_base=solver.arm_in_base.to_dict(),
        tcp_in_flange=solver.tool.to_dict(),ur_parameters=list(solver.parameters),
        fixed_joint_values=fixed,first_target=targets[0].to_dict(),
        first_base=proposal['base_planes'][0].to_dict())
    return proposal

"""Smooth adaptive mobile-base proposals and ordered configuration validation.

No automatic constraint relaxation. Shared APIs have no timeout;
callers may provide a cooperative cancel_check callback.
"""
from collections import Counter
from contextlib import ExitStack
from time import perf_counter
import math
import numpy as np
from .recording import recorded, current_run, event, metric
from .execution import high_qos
from .geometry import Plane, as_plane
from .xy_smoothing import smooth_xy
from .xy_centerline import centerline_xy
from .xy_offset import centerline_offset_frames, offset_frames
from .xy_sections import prepare_sections
from .stationary_region import StationaryRegion
from .planning import candidates, rotation_offsets
from .graph import shortest_path


@recorded
def generate_base_path(targets, *, max_xy_deviation=.25, normal_offset=1.0, tangent_offset=1.3,
                       geometry_mode='auto', geometry_options=None, _geometry_out=None):
    targets = [as_plane(t) for t in targets]
    smoothing = smooth_xy([t.origin for t in targets], max_xy_deviation)
    guide = centerline_xy(smoothing['curve'])
    frames = offset_frames(smoothing['curve'],[t.xaxis for t in targets],[t.yaxis for t in targets],0.,0.)
    geometry, diagnostics = prepare_sections([t.origin for t in targets], smoothing['curve'], guide,
        frames, mode=geometry_mode, options=geometry_options)
    if any(s['kind']!='arc' for s in diagnostics['path_sections']):
        frames = centerline_offset_frames(guide['curve'], guide['mapped_points'],
            [t.xaxis for t in targets], [t.yaxis for t in targets],
            x_offset=-normal_offset, y_offset=tangent_offset, pass_points=smoothing['curve'])
        geometry.x, geometry.y = frames['x_axes'], frames['y_axes']
    bases = geometry.frames([normal_offset, tangent_offset])
    if _geometry_out is not None:
        _geometry_out.append(geometry)
    return dict(base_planes=bases, smoothing=smoothing, centerline=guide,
                target_indices=list(range(len(targets))), **diagnostics)


@recorded
def validate_base_path(targets, bases, *, solver, world, joint_ranges, periodic,
                       current_pose=None, rotation_steps=16, max_joint_step=2.5, max_base_step=.25,
                       max_yaw_step=.25, max_reach_xy=1.75, collision_options=None,
                       time_intervals=None, max_base_speed=None, max_yaw_speed=None,
                       max_joint_speed=None, cancel_check=None, progress=None, _cache=None,
                       _complete_collision_layers=False):
    """Find the exact shortest arm path with configuration-only collision checking.

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
    # Cache lifetime is one adaptive planning call, with a fixed robot/world,
    # calibration, ranges and collision settings. Pose keys retain full precision.
    cache = {} if _cache is None else _cache
    ik_cache = cache.setdefault('ik', {})
    collision_cache = cache.setdefault('collision', {})
    layers, diagnostics, layer_angles, keys = [], [], [], []
    timing = dict(placement_seconds=0., ik_seconds=0., joint_filter_seconds=0.,
                  collision_seconds=0., graph_seconds=0.)
    ik_hits = collision_hits = collision_checks = 0
    def pose_key(target, base):
        return target.matrix.tobytes(), base.matrix.tobytes()
    def joint_key(q):
        return tuple(np.round(q, 12))
    event('mobile.validation_policy', collision_order='after_graph', check_edges=False,
          failed_layer_policy='complete_exact_checks' if _complete_collision_layers else 'selected_nodes_only',
          rotation_steps=int(rotation_steps), targets=len(targets),
          selection='full-case comparison 20260921: before_graph 100.186 s; after_graph 9.521 s; identical cost')
    for i,(target,base) in enumerate(zip(targets,bases)):
        check()
        key = pose_key(target,base)
        keys.append(key)
        if key in ik_cache:
            rows, angles, stored = ik_cache[key]
            detail = dict(stored, index=i)
            ik_hits += 1
        else:
            tick = perf_counter()
            placement=StationaryRegion([target],solver.arm_in_base,max_distance=max_reach_xy,projected=True).metrics(base)
            detail=dict(index=i,placement=placement,raw_ik=None,within_joint_limits=None)
            rows, angles = [], []
            if not np.allclose(base.zaxis,[0,0,1],atol=1e-10) or abs(base.origin[2])>1e-10:
                detail.update(state='placement_rejection',reason='Base is not upright at ground Z=0')
            elif not placement['geometry_valid']:
                detail.update(state='placement_rejection',reason='Negative target-Z side or calibrated arm-origin XY reach rule',reach_limit=max_reach_xy)
            elif not world.is_base_valid(base,clearance=clearance):
                detail.update(state='body_collision',reason=world.last_failure)
            timing['placement_seconds'] += perf_counter()-tick
            if 'state' not in detail:
                angles_by_q = {}
                def tracked_solver(rotated, footprint):
                    check()
                    angle=math.atan2(np.dot(target.yaxis,rotated.xaxis),np.dot(target.xaxis,rotated.xaxis))%(2*math.pi)
                    solutions=list(solver(rotated,footprint))
                    for q in solutions:
                        angles_by_q.setdefault(joint_key(q),angle)
                    return solutions
                tracked_solver.revolute_joints=getattr(solver,'revolute_joints',())
                raw_stats = {}
                rows,_,_=candidates(target,base,tracked_solver,offsets,None,joint_ranges,stats=raw_stats)
                detail.update(raw_ik=raw_stats['raw_ik'],within_joint_limits=raw_stats['within_joint_limits'])
                timing['ik_seconds'] += raw_stats['ik_seconds']
                timing['joint_filter_seconds'] += raw_stats['joint_expansion_seconds']
                angles = [angles_by_q[joint_key(q)] for q in rows]
                detail['state'] = 'configuration_untested' if rows else ('no_ik' if not raw_stats['raw_ik'] else 'joint_limit_rejection')
            ik_cache[key] = rows, angles, dict(detail)
        detail.update(rotation_steps=int(rotation_steps), collision_free=None,
                      collision_checks=0, collision_rejections=0, rejection_reasons={})
        layers.append(rows);layer_angles.append(angles);diagnostics.append(detail)
        if progress is not None and ((i+1)%100==0 or i+1==len(targets)):
            progress(dict(stage='ik_candidates',tested=i+1,total=len(targets),ik_cache_hits=ik_hits,
                          elapsed_seconds=perf_counter()-started))
    per_layer = None
    if max_joint_speed is not None:
        per_layer = np.tile(step,(len(targets),1))
        first = int(current_pose is None)
        per_layer[first:] = np.minimum(step, durations[:,None]*np.broadcast_to(max_joint_speed,(6,)))
    checked_nodes = {}
    def node_valid(i,j):
        nonlocal collision_hits, collision_checks
        if (i,j) in checked_nodes:
            return checked_nodes[i,j]
        check()
        key = keys[i], tuple(layers[i][j])
        if key in collision_cache:
            accepted, reason = collision_cache[key]
            collision_hits += 1
        else:
            tick = perf_counter()
            accepted = world.is_valid(layers[i][j],bases[i],clearance=clearance)
            reason = None if accepted else (world.last_failure or 'Configuration collision')
            timing['collision_seconds'] += perf_counter()-tick
            collision_checks += 1
            collision_cache[key] = accepted, reason
        d = diagnostics[i]
        d['collision_checks'] += 1
        if accepted:
            d['collision_free'] = (d['collision_free'] or 0)+1
            d['state'] = 'feasible_state'
        else:
            d['collision_rejections'] += 1
            reasons = d['rejection_reasons']
            reasons[reason] = reasons.get(reason,0)+1
            if d['collision_rejections'] == len(layers[i]):
                d['state'] = 'configuration_collision'
                d['collision_free'] = 0
        checked_nodes[i,j] = accepted
        return accepted
    completed_layers = set()
    def rejection_group(i,j):
        # A tight arc can reject most branches at most targets. Finish exact
        # checks in a failed layer once, rather than rebuild the entire graph
        # separately for each colliding branch. No pose equivalence is assumed.
        rejected = [k for k in range(len(layers[i])) if not node_valid(i,k)]
        completed_layers.add(i)
        if progress is not None and len(completed_layers)%100==0:
            progress(dict(stage='collision_candidates',completed_layers=len(completed_layers),
                          collision_checks=collision_checks,total=len(targets),
                          elapsed_seconds=perf_counter()-started))
        return rejected
    graph_stats = {}
    solved = None
    initial_failure = None
    if current_pose is not None:
        from .planning import _in_ranges, normalize_joint_ranges
        if len(current_pose)!=6 or not np.isfinite(current_pose).all():
            raise ValueError('current_pose requires six finite radians')
        if not _in_ranges(current_pose, normalize_joint_ranges(joint_ranges,6)):
            initial_failure = 'Starting configuration exceeds joint limits'
        elif not world.is_valid(current_pose,bases[0],clearance=clearance):
            initial_failure = world.last_failure or 'Starting configuration collision'
    if all(len(rows) for rows in layers) and not transitions and initial_failure is None:
        if progress is not None:
            progress(dict(stage='joint_graph',total=len(targets),elapsed_seconds=perf_counter()-started))
        solved=shortest_path(layers,start=current_pose,periodic=periodic,max_step=max_joint_step,
            step_limits=per_layer,node_valid=node_valid,stats=graph_stats,count_paths=False,
            node_rejection_group=rejection_group if _complete_collision_layers else None,
            revolute_joints=getattr(solver,'revolute_joints',None),cancel_check=cancel_check)
        timing['graph_seconds'] = graph_stats['graph_seconds']
    configurations = solved.configurations if solved is not None else []
    valid = len(configurations)==len(targets)
    selected_angles = [layer_angles[i][j] for i,j in enumerate(solved.indices)] if valid else []
    selected_targets = [t.rotated_z(a) for t,a in zip(targets,selected_angles)]
    unreachable = [d['index'] for d in diagnostics if d['state'] in
                   ('placement_rejection','body_collision','no_ik','joint_limit_rejection','configuration_collision')]
    unchecked = [d['index'] for d in diagnostics if d['state']=='configuration_untested']
    failure_layer = 0 if initial_failure else (solved.failure_layer if solved is not None else None)
    disconnected_detail = None
    if failure_layer is not None:
        previous = ([current_pose] if failure_layer==0 and current_pose is not None else
                    [layers[failure_layer-1][j] for j in solved.reachable_indices] if failure_layer>0 else [])
        best = None
        limit = step if per_layer is None else per_layer[failure_layer]
        if layers[failure_layer]:
            for q0 in previous:
                delta=np.asarray(layers[failure_layer])-q0
                delta[:,periodic]=(delta[:,periodic]+math.pi)%(2*math.pi)-math.pi
                score=np.max(abs(delta)/np.maximum(limit,1e-15),axis=1)
                j=int(np.argmin(score))
                if best is None or score[j]<best[0]:best=(float(score[j]),abs(delta[j]).tolist())
        disconnected_detail=dict(from_target=failure_layer-1,to_target=failure_layer,
            minimum_joint_step_limit_ratio=None if best is None else best[0],
            joint_deltas_at_nearest_pair=None if best is None else best[1],joint_step_limits=limit.tolist(),
            reason=initial_failure or 'No connected path under configuration and movement constraints')
    counts=dict(Counter(d['state'] for d in diagnostics))
    if valid:
        status='Validated {} targets and the exact shortest connected arm path; configuration collisions checked, transitions not collision-checked.'.format(len(targets))
    elif initial_failure:
        status=initial_failure+'. No validated arm path.'
    elif unreachable:
        i=unreachable[0];status='Proposal failed at target {}: {}. No validated arm path.'.format(i,diagnostics[i]['state'])
    elif transitions:
        status='Proposal failed at base transition {} -> {}.'.format(transitions[0]['from_target'],transitions[0]['to_target'])
    else:
        status='Disconnected arm path at transition {} -> {}.'.format(failure_layer-1 if failure_layer is not None else '?',failure_layer)
    timing['validation_seconds'] = perf_counter()-started
    for name,value in timing.items():metric('mobile.'+name,value,'s')
    for name,value in dict(ik_cache_hits=ik_hits,collision_cache_hits=collision_hits,collision_checks=collision_checks).items():
        metric('mobile.'+name,value)
    event('mobile.validation_result',valid=valid,state_counts=counts,unchecked_targets=unchecked,
          unreachable_targets=unreachable,disconnected=disconnected_detail)
    return dict(base_planes=bases,configurations=configurations,fabrication_validated=valid,
        optimality_certified=valid,collision_order='after_graph',graph_stats=graph_stats,
        ik_cache_hits=ik_hits,collision_cache_hits=collision_hits,rotation_steps=int(rotation_steps),
        selected_tcp_rotations=selected_angles,selected_target_planes=selected_targets,
        status=status,target_diagnostics=diagnostics,state_counts=counts,unreachable_points=unreachable,
        unchecked_points=unchecked,transition_failures=transitions,disconnected_target=failure_layer,
        disconnected_detail=disconnected_detail,edge_rejection_reasons={},initial_state_failure=initial_failure,
        max_joint_step=max_joint_step,path_length=solved.cost if valid else None,
        collision_check_applied=True,check_edges=False,collision_validation='configurations_only',
        collision_failed_layer_policy='complete_exact_checks' if _complete_collision_layers else 'selected_nodes_only',
        speed_checked=any(v is not None for v in (max_base_speed,max_yaw_speed,max_joint_speed)),
        timings=timing,validation_seconds=timing['validation_seconds'])


@recorded
@high_qos
def plan_base_path(targets, *, solver, world, joint_ranges, periodic, adapt_offsets=True,
                   max_xy_deviation=.25, normal_offset=1.0, tangent_offset=1.3,
                   base_yaw_degrees=0., geometry_mode='auto', geometry_options=None, **limits):
    """Numeric mobile workflow shared by GH and capture replay; lengths in metres."""
    from .mobile_adaptation import repair_offsets
    base_yaw_degrees = float(base_yaw_degrees)
    if not math.isfinite(base_yaw_degrees):
        raise ValueError('base_yaw_degrees must be finite')
    angle = math.radians(base_yaw_degrees % 360.)
    c, s = math.cos(angle), math.sin(angle)
    def orient(bases):
        # Rotate at each origin; keep offset repairs in the centerline axes.
        if angle == 0.:
            return bases
        return [Plane(b.origin, c*b.xaxis+s*b.yaxis, -s*b.xaxis+c*b.yaxis) for b in bases]
    started = perf_counter()
    targets = [as_plane(t) for t in targets]
    tick = perf_counter()
    prepared = []
    proposal = generate_base_path(targets,max_xy_deviation=max_xy_deviation,
        normal_offset=normal_offset,tangent_offset=tangent_offset, geometry_mode=geometry_mode,
        geometry_options=geometry_options, _geometry_out=prepared)
    if proposal.get('unresolved_sections'):
        raise ValueError('Unresolved strong curvature in sections '+str(proposal['unresolved_sections'])+
                         '; inspect generate_base_path diagnostics or adjust geometry_options')
    geometry_seconds = perf_counter()-tick
    has_arcs = any(s['kind']=='arc' for s in proposal.get('path_sections',[]))
    cache = {}
    totals = Counter()
    def validate(bases):
        result = validate_base_path(targets,orient(bases),solver=solver,world=world,joint_ranges=joint_ranges,
            periodic=periodic,_cache=cache,
            _complete_collision_layers=has_arcs,**limits)
        totals.update(result['timings'])
        return result
    result = validate(proposal['base_planes'])
    applied = np.tile([normal_offset,abs(tangent_offset)],(len(targets),1))
    attempts = []
    repair_seconds = 0.
    if adapt_offsets and not result['fabrication_validated'] and not result['initial_state_failure']:
        def placement_probe(indices,bases):
            oriented = orient(bases)
            # Screen the entire anchor group cheaply before generating any IK.
            # Otherwise a late body collision wastes all earlier anchor IK.
            for i,b in zip(indices,oriented):
                tick = perf_counter()
                placed = StationaryRegion([targets[i]],solver.arm_in_base,
                    max_distance=limits.get('max_reach_xy',1.75),projected=True).metrics(b)['geometry_valid']
                totals['placement_seconds'] += perf_counter()-tick
                if not placed:
                    return False
                tick = perf_counter()
                clear = world.is_base_valid(b,clearance=(limits.get('collision_options') or {}).get('clearance',0.))
                totals['collision_seconds'] += perf_counter()-tick
                if not clear:
                    return False
            return True
        def probe(indices,bases):
            options = {k:v for k,v in limits.items() if k in
                       ('rotation_steps','max_reach_xy','collision_options','cancel_check')}
            # Anchor tests are individual placement checks, not transitions over
            # skipped TCPs. Full original-index connectivity is checked afterward.
            if not placement_probe(indices,bases):
                return False
            for i,b in zip(indices,orient(bases)):
                p = validate_base_path([targets[i]],[b],solver=solver,world=world,
                    joint_ranges=joint_ranges,periodic=periodic,_cache=cache,**options)
                totals.update(p['timings'])
                if not p['fabrication_validated']:
                    return False
            return True
        tick = perf_counter()
        extent = limits.get('max_reach_xy',1.75)+np.linalg.norm(solver.arm_in_base.origin[:2])+max_xy_deviation
        result,applied,attempts = repair_offsets(targets,proposal['base_planes'],result,
            validate=validate,probe=probe,normal_offset=normal_offset,tangent_offset=tangent_offset,
            search_extent=float(extent),cancel_check=limits.get('cancel_check'),
            build_frames=prepared[0].frames if prepared else None,prefer_progress=has_arcs)
        repair_seconds = perf_counter()-tick
    if tangent_offset < 0:applied[:,1] *= -1
    proposal.update(result)
    proposal.update(applied_offsets=applied,repair_attempts=attempts,adapt_offsets=bool(adapt_offsets),
                    base_yaw_degrees=base_yaw_degrees,
                    excluded_collision_links=sorted(getattr(world,'excluded_collision_links',())),
                    base_collision_geometry=getattr(world,'base_collision_geometry',{}))
    proposal['timings'] = dict(totals,geometry_seconds=geometry_seconds,repair_seconds=repair_seconds,
                              planning_seconds=perf_counter()-started)
    if attempts and not result['fabrication_validated']:
        proposal['status'] += ' Sampled offset repair search exhausted; this is not proof of infeasibility.'
    run = current_run()
    proposal['research_run'] = str(run.path) if run is not None else None
    event('mobile.plan_result',valid=result['fabrication_validated'],targets=len(targets),
          repair_attempts=len(attempts),timings=proposal['timings'],
          geometry_mode=geometry_mode,path_sections=proposal.get('path_sections',[]))
    return proposal


@recorded
@high_qos
def plan_mobile_base(robot, target_planes, *, units_to_metres=1., max_xy_deviation=.25,
                     normal_offset=1.0, tangent_offset=1.3, adapt_offsets=True, current_pose=None, arm_in_base=None,
                     arm_joint_names=None, fixed_joint_values=None, joint_ranges=None,
                     collision_meshes=(), collision_options=None, group=None, parameters=None,
                     scene=None, cancel_check=None, **limits):
    """Robot boundary: inputs planes/meshes in model units; geometric settings metres.

    Robot/tool and fixed joints are metres/radians. Targets may rotate about local
    TCP Z (16 samples by default), preserving positions and extrusion directions. Configuration collision validation is mandatory; swept edges are not checked.
    """
    from .robot_adapter import kinematics_from_robot,resolve_arm_joint_names,configuration_from_values,_active_tool
    from .robot_planning import json_input
    from .collision import PybulletServer
    if robot is None: raise ValueError('Connect robot with its calibrated active tool')
    if not math.isfinite(units_to_metres) or units_to_metres<=0: raise ValueError('units_to_metres must be positive')
    if _active_tool(robot,group) is None: raise ValueError('Attach the calibrated active tool before mobile validation')
    setup_started=perf_counter()
    targets=[as_plane(t,units_to_metres) for t in target_planes]
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
    allowed={'gui','allowed_pairs','package_paths','asset_root','ground_z','support_links','joint_resolution','base_resolution','yaw_resolution','clearance','check_static_self_collisions','exclude_gps','excluded_collision_links','base_collision_model'}
    if set(options)-allowed: raise ValueError('Unknown collision options: '+str(set(options)-allowed))
    with ExitStack() as stack:
        if scene is not None:
            if collision_meshes or options: raise ValueError('External scene must already contain environment; omit collision_meshes/options')
            world=scene
            if list(world.joint_names)!=names: raise ValueError('Collision scene joint order differs')
        else:
            constructor={k:options[k] for k in ('gui','allowed_pairs','package_paths','asset_root','check_static_self_collisions','exclude_gps','excluded_collision_links','base_collision_model') if k in options}
            constructor.setdefault('check_static_self_collisions',False)
            constructor.setdefault('exclude_gps',True)
            constructor.setdefault('base_collision_model','auto')
            constructor['fixed_joint_values']=fixed
            world=stack.enter_context(PybulletServer(robot=robot,joint_names=names,**constructor))
            for mesh in collision_meshes: world.add_mesh(mesh,scale=units_to_metres)
            if 'ground_z' in options: world.add_ground(options['ground_z'],support_links=options.get('support_links',()))
        world.set_fixed_joints(fixed)
        setup_seconds=perf_counter()-setup_started
        proposal=plan_base_path(targets,solver=solver,world=world,joint_ranges=ranges,
            periodic=[joints[n].type==1 for n in names],current_pose=start,collision_options=options,
            adapt_offsets=adapt_offsets,max_xy_deviation=max_xy_deviation,normal_offset=normal_offset,
            tangent_offset=tangent_offset,cancel_check=cancel_check,**limits)
        proposal['excluded_collision_links'] = sorted(getattr(world,'excluded_collision_links',()))
    proposal['timings']['setup_seconds']=setup_seconds
    output_names=[n for n,j in joints.items() if j.type==2 and n not in names]+names
    proposal['configuration_objects']=[configuration_from_values([dict(fixed,**dict(zip(names,q)))[n] for n in output_names],output_names,[joints[n].type for n in output_names]) for q in proposal['configurations']]
    proposal.update(arm_in_base=solver.arm_in_base,fixed_joint_values=fixed,mounting_source=solver.mounting_source)
    proposal['effective_settings']=dict(units_to_metres=units_to_metres,
        geometry_mode=proposal.get('geometry_mode','auto'), geometry_options=proposal.get('geometry_options',{}),
        excluded_collision_links=proposal['excluded_collision_links'],
        base_yaw_degrees=proposal['base_yaw_degrees'],
        rotation_steps=proposal['rotation_steps'],solver=type(solver).__name__,
        mounting_source=solver.mounting_source,arm_in_base=solver.arm_in_base.to_dict(),
        tcp_in_flange=solver.tool.to_dict(),ur_parameters=list(solver.parameters),
        fixed_joint_values=fixed,first_target=targets[0].to_dict(),
        first_base=proposal['base_planes'][0].to_dict())
    return proposal

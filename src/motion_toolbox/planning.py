"""Shared candidate evaluation for stationary, prescribed-base and mobile planning."""
from motion_toolbox.recording import recorded, current_run, metric, event
import json
import math
import itertools
from time import perf_counter
import numpy as np
from .geometry import Plane, as_plane
from .graph import shortest_path
from .kinematics.solver import URKinematics


def rotation_offsets(mode=False, step=5, steps=35, cw=0, ccw=0):
    mode = str(mode).lower()
    if mode in ('false', 'none', 'off', '0'):
        return [0.0]
    if mode == 'n_steps':
        if int(steps) != steps or steps < 1:
            raise ValueError('rotation_steps must be a positive integer')
        return [2*math.pi*i/steps for i in range(steps)]
    if mode == 'step_angle':
        if not all(math.isfinite(v) for v in [step, cw, ccw]) or step <= 0 or cw < 0 or ccw < 0:
            raise ValueError('Positive finite step and nonnegative rotation bounds required')
        return [math.radians(-ccw+i*step) for i in range(int((cw+ccw)/step+1e-9)+1)]
    raise ValueError('Unknown rotation mode')


def normalize_joint_ranges(ranges, dimension=None):
    if ranges is None:
        return None
    if dimension is not None and len(ranges) > dimension:
        raise ValueError('More joint ranges than joints')
    result = []
    for limits in ranges:
        if limits is None:
            result.append(None)
            continue
        if len(limits) != 2:
            raise ValueError('Each joint range must be None or [min, max]')
        lo, hi = (None if v is None else float(v) for v in limits)
        if any(v is not None and not math.isfinite(v) for v in (lo, hi)):
            raise ValueError('Joint bounds must be finite or None')
        if lo is not None and hi is not None and lo > hi:
            raise ValueError('Reversed joint range')
        result.append((lo, hi))
    return result


def _in_ranges(q, ranges):
    if ranges is None:
        return True
    if len(ranges) > len(q):
        raise ValueError('More joint ranges than joints')
    for value, limits in zip(q, ranges):
        if limits is None:
            continue
        lo, hi = limits
        if lo is not None and hi is not None and lo > hi:
            raise ValueError('Reversed joint range')
        if (lo is not None and value < lo) or (hi is not None and value > hi):
            return False
    return True


@recorded
def candidates(target, base, ik_solver, offsets, collision=None, joint_ranges=None, *, stats=None):
    """Return unique feasible joint vectors and diagnostic counts."""
    if stats is None and current_run() is not None:
        stats = {}
    started = perf_counter()
    target, base = as_plane(target), as_plane(base)
    joint_ranges = normalize_joint_ranges(joint_ranges)
    all_q, seen = [], set()
    for angle in offsets:
        for q in ik_solver(target.rotated_z(angle), base):
            q = np.asarray(q, dtype=float)
            if q.ndim != 1 or q.size == 0 or not np.isfinite(q).all():
                raise ValueError('IK returned an invalid configuration')
            key = tuple(np.round(q, 12))
            if key not in seen:
                seen.add(key)
                all_q.append(q.tolist())
    ik_finished = perf_counter()
    ranged = []
    for q in all_q:
        # Analytic IK returns principal angles. Enumerate valid revolutions for
        # explicitly bounded joints so limits such as [-2*pi, 0] remain usable.
        alternatives = []
        for j, value in enumerate(q):
            bounds = joint_ranges[j] if joint_ranges is not None and j < len(joint_ranges) else None
            if j in getattr(ik_solver, 'revolute_joints', ()) and bounds is not None and bounds[0] is not None and bounds[1] is not None:
                lo, hi = bounds
                if not math.isfinite(lo) or not math.isfinite(hi) or lo > hi:
                    raise ValueError('Joint bounds must be finite and ordered')
                alternatives.append([value+2*math.pi*k for k in range(math.ceil((lo-value)/(2*math.pi)), math.floor((hi-value)/(2*math.pi))+1)])
            else:
                alternatives.append([value])
        ranged.extend(list(v) for v in itertools.product(*alternatives) if _in_ranges(v, joint_ranges))
    expansion_finished = perf_counter()
    valid = ranged
    failures = {}
    collision_checks = 0
    collision_rejections = 0
    collision_cache_hits = 0
    if collision is not None:
        valid = []
        checker = getattr(collision, '__self__', None)
        # Robot/CLI entry points bind clearance using functools.partial.
        from functools import partial
        if isinstance(collision, partial) and not collision.args:
            checker = getattr(collision.func, '__self__', None)
        cache_key = getattr(checker, 'configuration_cache_key', None)
        cache = {}
        for q in ranged:
            key = cache_key(q) if cache_key is not None else None
            if key is not None and key in cache:
                collision_cache_hits += 1
                accepted, reason = cache[key]
            else:
                collision_checks += 1
                accepted = collision(q, base)
                if not accepted:
                    collision_rejections += 1
                reason = getattr(checker, 'last_failure', None) or 'collision checker rejected configuration'
                if key is not None:
                    cache[key] = accepted, reason
            if accepted:
                valid.append(q)
            elif stats is not None:
                failures[reason] = failures.get(reason, 0)+1
    if stats is not None:
        stats.update(raw_ik=len(all_q), within_joint_limits=len(ranged),
                     collision_checks=collision_checks, collision_rejections=collision_rejections,
                     collision_cache_hits=collision_cache_hits,
                     collision_free=len(valid), rejection_reasons=failures,
                     ik_seconds=ik_finished-started,
                     joint_expansion_seconds=expansion_finished-ik_finished,
                     collision_seconds=perf_counter()-expansion_finished)
        for name in ('raw_ik', 'within_joint_limits', 'collision_free', 'collision_checks', 'collision_rejections', 'collision_cache_hits'):
            metric(name, stats[name])
        for name in ('ik_seconds', 'joint_expansion_seconds', 'collision_seconds'):
            metric(name, stats[name], 's')
        event('candidate.filters', **stats)
    return valid, len(all_q), len(valid)


@recorded
def calculate_partial_trajectory(current_pose, list_of_targets, number_of_nodes_to_calculate=None,
        base_planes=None, rotation_mode=False, rotation_angle_deg=5, rotation_steps=35,
        angle_cw_deg=0, angle_ccw_deg=0, *, ik_solver=None, tool=None, arm_in_base=None,
        collision=None, enable_collision_check=False, joint_ranges=None, dont_build_graph=False,
        max_joint_step=2.5, periodic=None, weights=None, edge_valid=None, transition_check=None, start_base=None,
        ik_solutions_output_path=None, path_builder_iterations=None):
    """Plan a prefix, preserving one result slot per TCP target.

    base_planes describe the URDF root/footprint in world coordinates. Configure
    arm_in_base when it differs from the analytic arm origin. collision(q, base)
    returns True for a valid pose; use a persistent PybulletServer.is_valid.
    path_builder_iterations is accepted for ROS migration; solve is always exact.
    """
    joint_ranges = normalize_joint_ranges(joint_ranges, len(current_pose) if current_pose is not None else None)
    total = len(list_of_targets)
    n = total if number_of_nodes_to_calculate is None else number_of_nodes_to_calculate
    if not isinstance(n, int) or n < 0:
        raise ValueError('Prefix length must be a nonnegative integer')
    n = min(n, total)
    targets = [as_plane(t) for t in list_of_targets[:n]]
    if base_planes is None:
        bases = [Plane.world_xy()] * n
    elif len(base_planes) == 1:
        bases = [as_plane(base_planes[0])] * n
    elif len(base_planes) == total:
        bases = [as_plane(b) for b in base_planes[:n]]
    else:
        raise ValueError('Provide one base or one base per target')
    if enable_collision_check and collision is None:
        raise ValueError('Collision checking requested without a configured collision checker')
    offsets = rotation_offsets(rotation_mode, rotation_angle_deg, rotation_steps, angle_cw_deg, angle_ccw_deg)
    solver = ik_solver if ik_solver is not None else URKinematics(tool=tool, arm_in_base=arm_in_base)
    layers, before, after, diagnostics = [], [], [], []
    candidate_started = perf_counter()
    for target, base in zip(targets, bases):
        stats = {}
        q, a, b = candidates(target, base, solver, offsets, collision, joint_ranges, stats=stats)
        diagnostics.append(stats)
        layers.append(q)
        before.append(a)
        after.append(b)
    candidate_seconds = perf_counter()-candidate_started
    unreachable = [i for i, q in enumerate(layers) if not q]
    result = dict(configurations=[], path_length=float('inf'), num_nodes_computed=n,
        ik_solutions_per_node=layers, rotation_candidates_per_node=[len(offsets)]*n,
        unreachable_points=unreachable, collision_check_applied=collision is not None,
        solution_counts_before_collision=before, solution_counts_after_collision=after,
        target_diagnostics=diagnostics,
        timings=dict(candidate_seconds=candidate_seconds, graph_seconds=0.0,
                     **{key: sum(d[key] for d in diagnostics) for key in
                        ('ik_seconds', 'joint_expansion_seconds', 'collision_seconds')}))
    if not dont_build_graph and not unreachable:
        graph_started = perf_counter()
        def check_edge(i, a, b):
            if edge_valid is not None and not edge_valid(i, a, b):
                return False
            if transition_check is None:
                return True
            q0 = current_pose if i == 0 else layers[i-1][a]
            b0 = (as_plane(start_base) if start_base is not None else bases[0]) if i == 0 else bases[i-1]
            return transition_check(q0, b0, layers[i][b], bases[i])
        solved = shortest_path(layers, start=current_pose, max_step=max_joint_step,
                               periodic=periodic, weights=weights,
                               count_paths=False, revolute_joints=getattr(solver, 'revolute_joints', None),
                               edge_valid=check_edge if edge_valid is not None or transition_check is not None else None)
        result.update(configurations=solved.configurations, path_length=solved.cost)
        result['timings']['graph_seconds'] = perf_counter()-graph_started
    if ik_solutions_output_path is not None:
        payload = dict(result)
        payload['path_length'] = result['path_length'] if math.isfinite(result['path_length']) else None
        with open(ik_solutions_output_path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, allow_nan=False)
    return result


plan_targets_with_base_positions = calculate_partial_trajectory

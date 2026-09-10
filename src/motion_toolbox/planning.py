"""Shared candidate evaluation for stationary, prescribed-base and mobile planning."""
import json
import math
import itertools
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


def candidates(target, base, ik_solver, offsets, collision=None, joint_ranges=None):
    """Return unique feasible joint vectors and diagnostic counts."""
    target, base = as_plane(target), as_plane(base)
    all_q, seen = [], set()
    for angle in offsets:
        for q in ik_solver(target.rotated_z(angle), base):
            q = np.asarray(q, dtype=float)
            if q.ndim != 1 or not np.isfinite(q).all():
                raise ValueError('IK returned an invalid configuration')
            key = tuple(np.round(q, 12))
            if key not in seen:
                seen.add(key)
                all_q.append(q.tolist())
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
    valid = ranged if collision is None else [q for q in ranged if collision(q, base)]
    return valid, len(all_q), len(valid)


def calculate_partial_trajectory(current_pose, list_of_targets, number_of_nodes_to_calculate=None,
        base_planes=None, rotation_mode=False, rotation_angle_deg=5, rotation_steps=35,
        angle_cw_deg=0, angle_ccw_deg=0, *, ik_solver=None, tool=None, arm_in_base=None,
        collision=None, enable_collision_check=False, joint_ranges=None, dont_build_graph=False,
        max_joint_step=2.5, periodic=None, weights=None, edge_valid=None, transition_check=None,
        ik_solutions_output_path=None, path_builder_iterations=None):
    """Plan a prefix, preserving one result slot per TCP target.

    base_planes describe the URDF root/footprint in world coordinates. Configure
    arm_in_base when it differs from the analytic arm origin. collision(q, base)
    returns True for a valid pose; use a persistent PybulletServer.is_valid.
    path_builder_iterations is accepted for ROS migration; solve is always exact.
    """
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
    layers, before, after = [], [], []
    for target, base in zip(targets, bases):
        q, a, b = candidates(target, base, solver, offsets, collision, joint_ranges)
        layers.append(q)
        before.append(a)
        after.append(b)
    unreachable = [i for i, q in enumerate(layers) if not q]
    result = dict(configurations=[], path_length=float('inf'), num_nodes_computed=n,
        ik_solutions_per_node=layers, rotation_candidates_per_node=[len(offsets)]*n,
        unreachable_points=unreachable, collision_check_applied=collision is not None,
        solution_counts_before_collision=before, solution_counts_after_collision=after)
    if not dont_build_graph and not unreachable:
        def check_edge(i, a, b):
            if edge_valid is not None and not edge_valid(i, a, b):
                return False
            if transition_check is None:
                return True
            q0 = current_pose if i == 0 else layers[i-1][a]
            b0 = bases[0] if i == 0 else bases[i-1]
            return transition_check(q0, b0, layers[i][b], bases[i])
        solved = shortest_path(layers, start=current_pose, max_step=max_joint_step,
                               periodic=periodic, weights=weights,
                               edge_valid=check_edge if edge_valid is not None or transition_check is not None else None)
        result.update(configurations=solved.configurations, path_length=solved.cost)
    if ik_solutions_output_path is not None:
        payload = dict(result)
        payload['path_length'] = result['path_length'] if math.isfinite(result['path_length']) else None
        with open(ik_solutions_output_path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, indent=2, allow_nan=False)
    return result


plan_targets_with_base_positions = calculate_partial_trajectory

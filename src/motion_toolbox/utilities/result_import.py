"""Restore stationary benchmark results without planning or executing robot code."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from ..geometry import as_plane
from ..robot_adapter import configuration_from_values


def _path(value):
    return Path(str(value).strip().strip('"')).expanduser()


def _read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def load_stationary_result(result_path, *, case_folder=None):
    """Load the JSON produced by benchmark_adaptive_stationary (either strategy).

    Planes and fixed prismatic values remain in metres; arm values are radians.
    ``valid`` describes saved verification only, never the current Rhino scene.
    The capture hash and its manifest's URDF hash bind metadata to the run.
    No captured Python source is imported. Unsupported/partial data cannot emit
    trajectory outputs. A saved unsuccessful search returns diagnostics only.
    """
    path = _path(result_path).resolve()
    saved = _read(path)
    if not isinstance(saved, dict) or not {'case', 'case_sha256', 'path_complete',
            'configurations', 'base', 'selected_tcp_rotations'} <= saved.keys():
        raise ValueError('Expected a stationary benchmark result.json')
    case = _path(case_folder if case_folder is not None else saved['case'])
    if not case.is_absolute():
        case = path.parent / case
    if case.name == 'case.json':
        case = case.parent
    case = case.resolve()
    case_path = case / 'case.json'
    if not case_path.is_file():
        raise ValueError('Capture case.json not found; set case_folder to the exported capture folder')
    case_bytes = case_path.read_bytes()
    digest = hashlib.sha256(case_bytes).hexdigest()
    if digest != saved['case_sha256']:
        raise ValueError('Capture case.json hash does not match this result')
    capture = json.loads(case_bytes)
    replay = capture.get('replay', {})
    if capture.get('schema') != 'motion-mobile-case/1' or replay.get('units') != 'metres/radians':
        raise ValueError('Unsupported capture schema or units; expected metres/radians')
    verification = {key: saved.get(key) for key in (
        'final_collision_recheck', 'joint_limits_recheck', 'joint_steps_recheck',
        'max_fk_matrix_error', 'transition_collision_checked')}
    diagnostics = ['Imported saved data; no planning or collision checks were rerun.',
                   'Verification describes the captured scene, not the current Rhino scene.']
    if saved.get('transition_collision_checked') is not True:
        diagnostics.append('Swept transitions were not collision-checked.')
    restored = dict(source_result=saved, result_path=str(path), case_folder=str(case),
        case_sha256=digest, base_plane=None, base_planes=[], configurations=[],
        configuration_objects=[], selected_target_planes=[], selected_tcp_rotations=[],
        path_complete=False, valid=False, cost=None, verification=verification,
        diagnostics=diagnostics, target_count=len(replay['targets']))
    if saved['path_complete'] is not True:
        restored['status'] = 'Saved run has no complete path: ' + str(saved.get('status', 'unknown'))
        return restored
    n = len(replay['targets'])
    qs = np.asarray(saved['configurations'], dtype=float)
    angles = np.asarray(saved['selected_tcp_rotations'], dtype=float)
    if n == 0 or qs.shape != (n, 6) or not np.isfinite(qs).all():
        raise ValueError('Expected one finite six-joint configuration per captured target')
    if angles.shape != (n,) or not np.isfinite(angles).all():
        raise ValueError('Expected one finite selected TCP rotation per captured target')
    if saved['base'] is None:
        raise ValueError('Complete stationary result is missing its base plane')
    base = as_plane(saved['base'])
    selected = [as_plane(t).rotated_z(float(a)) for t, a in zip(replay['targets'], angles)]
    # Read joint types from XML only; do not load meshes or captured Python code.
    manifest = _read(case / 'manifest.json')
    urdf = (case / 'robot' / 'robot.urdf').read_bytes()
    if manifest.get('case.json') != digest or manifest.get('robot/robot.urdf') != hashlib.sha256(urdf).hexdigest():
        raise ValueError('Capture manifest does not match case.json or robot URDF')
    types = {'revolute': 0, 'continuous': 1, 'prismatic': 2, 'fixed': 3}
    joints = {j.attrib['name']: types.get(j.attrib.get('type')) for j in ET.fromstring(urdf).findall('joint')}
    arm = replay['arm_joint_names']
    fixed = replay['fixed_joint_values']
    if (len(arm) != 6 or len(set(arm)) != 6 or set(arm) & set(fixed)
            or any(joints.get(name) not in (0, 1) for name in arm)):
        raise ValueError('Capture must identify six distinct revolute arm joints, separate from fixed values')
    if any(name not in joints or joints[name] not in (0, 1, 2, 3) for name in fixed):
        raise ValueError('Captured fixed joint is missing or unsupported in the URDF')
    fixed_values = [float(v) for v in fixed.values()]
    if not all(math.isfinite(v) for v in fixed_values):
        raise ValueError('Fixed joint values must be finite')
    names = list(fixed) + list(arm)
    joint_types = [joints[name] for name in names]
    configurations = qs.tolist()
    objects = [configuration_from_values(fixed_values + q, names, joint_types) for q in configurations]
    cost = float(saved['cost'])
    if not math.isfinite(cost) or cost < 0:
        raise ValueError('Path cost must be finite and nonnegative')
    error = saved.get('max_fk_matrix_error')
    fk_passed = (isinstance(error, (int, float)) and not isinstance(error, bool)
                 and math.isfinite(error) and 0 <= error < 1e-6)
    verified = fk_passed and all(saved.get(key) is True for key in
        ('final_collision_recheck', 'joint_limits_recheck', 'joint_steps_recheck'))
    restored.update(base_plane=base, base_planes=[base] * n, configurations=configurations,
        configuration_objects=objects, selected_target_planes=selected,
        selected_tcp_rotations=angles.tolist(), path_complete=True, valid=verified, cost=cost,
        arm_joint_names=list(arm), fixed_joint_values=dict(fixed),
        status='Imported {} targets; saved waypoint verification {}.'.format(n, 'passed' if verified else 'not confirmed'))
    return restored

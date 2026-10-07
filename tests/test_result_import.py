import hashlib
import json
from pathlib import Path
import runpy

import numpy as np
import pytest

from motion_toolbox.geometry import Plane
from motion_toolbox.utilities.result_import import load_stationary_result
from test_grasshopper_entrypoints import gh  # Shared Rhino/DataTree boundary stubs.


@pytest.fixture
def saved_run(tmp_path):
    case = tmp_path / 'capture'
    (case / 'robot').mkdir(parents=True)
    targets = [Plane((1, 2, 3), (0, 1, 0), (0, 0, 1)).to_dict(),
               Plane((2, 3, 4), (1, 0, 0), (0, 1, 0)).to_dict()]
    replay = dict(units='metres/radians', targets=targets,
                  arm_joint_names=['j' + str(i) for i in range(6)],
                  fixed_joint_values={'lift': .25, 'wheel': -.3})
    capture = dict(schema='motion-mobile-case/1', replay=replay)
    (case / 'case.json').write_text(json.dumps(capture))
    urdf = '<robot name="fixture"><joint name="lift" type="prismatic"/><joint name="wheel" type="continuous"/>'
    urdf += ''.join('<joint name="j{}" type="revolute"/>'.format(i) for i in range(6)) + '</robot>'
    (case / 'robot' / 'robot.urdf').write_text(urdf)
    hashes = {name: hashlib.sha256((case / name).read_bytes()).hexdigest()
              for name in ('case.json', 'robot/robot.urdf')}
    (case / 'manifest.json').write_text(json.dumps(hashes))
    saved = dict(case=str(case), case_sha256=hashes['case.json'], path_complete=True,
        status='connected', configurations=[[.1 * i for i in range(6)], [.2 * i for i in range(6)]],
        selected_tcp_rotations=[np.pi / 2, -.2], base=Plane.world_xy().to_dict(), cost=1.5,
        final_collision_recheck=True, joint_limits_recheck=True, joint_steps_recheck=True,
        max_fk_matrix_error=1e-9, transition_collision_checked=False)
    path = tmp_path / 'result.json'
    path.write_text(json.dumps(saved))
    return path, saved, case, capture


def write_result(path, saved):
    path.write_text(json.dumps(saved))


def test_import_preserves_angles_joint_names_fixed_types_and_rotated_targets(saved_run):
    path, saved, case, capture = saved_run
    result = load_stationary_result(path)
    assert result['valid'] and result['path_complete']
    assert result['configurations'] == saved['configurations']
    assert len(result['base_planes']) == 2
    assert result['configuration_objects'][0].joint_names == ['lift', 'wheel'] + ['j' + str(i) for i in range(6)]
    assert list(result['configuration_objects'][0].joint_types) == [2, 1, 0, 0, 0, 0, 0, 0]
    assert result['configuration_objects'][1]['lift'] == .25
    assert result['configuration_objects'][1]['j5'] == 1.0
    # The first target's local X rotates toward its local Y (world +Z),
    # rather than about world Z. Its origin must remain unchanged.
    np.testing.assert_allclose(result['selected_target_planes'][0].xaxis, (0, 0, 1), atol=1e-15)
    np.testing.assert_allclose(result['selected_target_planes'][0].origin, (1, 2, 3))
    assert result['verification']['transition_collision_checked'] is False


def test_moved_capture_override_and_quoted_paths(saved_run):
    path, saved, case, _ = saved_run
    saved['case'] = 'missing/old/capture'
    write_result(path, saved)
    with pytest.raises(ValueError, match='case_folder'):
        load_stationary_result(path)
    assert load_stationary_result('"' + str(path) + '"', case_folder='capture/case.json')['valid']


@pytest.mark.parametrize('mutation, message', [
    (dict(configurations=[[0] * 6]), 'six-joint'),
    (dict(configurations=[[0] * 6, [float('nan')] * 6]), 'six-joint'),
    (dict(selected_tcp_rotations=[0]), 'TCP rotation'),
    (dict(selected_tcp_rotations=[0, float('inf')]), 'TCP rotation'),
    (dict(base=None), 'base plane'),
    (dict(cost=float('nan')), 'cost'),
    (dict(case_sha256='wrong'), 'hash'),
])
def test_inconsistent_complete_result_is_rejected(saved_run, mutation, message):
    path, saved, _, _ = saved_run
    saved.update(mutation)
    write_result(path, saved)
    with pytest.raises(ValueError, match=message):
        load_stationary_result(path)


def test_urdf_tampering_is_rejected(saved_run):
    path, _, case, _ = saved_run
    with (case / 'robot' / 'robot.urdf').open('a') as f:
        f.write(' ')
    with pytest.raises(ValueError, match='manifest'):
        load_stationary_result(path)


@pytest.mark.parametrize('key,value', [('units', 'millimetres'),
    ('arm_joint_names', ['j0'] * 6), ('fixed_joint_values', {'absent': 0})])
def test_invalid_captured_metadata_is_rejected(saved_run, key, value):
    path, saved, case, capture = saved_run
    capture['replay'][key] = value
    (case / 'case.json').write_text(json.dumps(capture))
    saved['case_sha256'] = hashlib.sha256((case / 'case.json').read_bytes()).hexdigest()
    manifest = json.loads((case / 'manifest.json').read_text())
    manifest['case.json'] = saved['case_sha256']
    (case / 'manifest.json').write_text(json.dumps(manifest))
    write_result(path, saved)
    with pytest.raises(ValueError):
        load_stationary_result(path)


@pytest.mark.parametrize('key', ['final_collision_recheck', 'joint_limits_recheck',
    'joint_steps_recheck', 'max_fk_matrix_error'])
def test_missing_saved_verification_never_becomes_valid(saved_run, key):
    path, saved, _, _ = saved_run
    del saved[key]
    write_result(path, saved)
    result = load_stationary_result(path)
    assert result['path_complete'] and not result['valid']
    assert result['verification'][key] is None


def test_failed_run_exposes_no_partial_trajectory(saved_run):
    path, saved, _, _ = saved_run
    saved.update(path_complete=False, status='no_connected_path')
    write_result(path, saved)
    result = load_stationary_result(path)
    assert not result['valid'] and not result['path_complete']
    for key in ('configurations', 'configuration_objects', 'base_planes', 'selected_target_planes'):
        assert result[key] == []
    assert result['base_plane'] is None


def test_component_scaling_data_tree_and_failure_clearing(saved_run, gh, monkeypatch):
    from motion_toolbox.geometry import as_plane
    monkeypatch.setattr('motion_toolbox.geometry.to_rhino', lambda p, scale=1: as_plane(p, scale))
    path, saved, _, _ = saved_run
    script = str(Path(__file__).resolve().parents[1] / 'examples' / 'grasshopper_import_result.py')
    inputs = dict(result_path=str(path), units_to_metres=.001)
    output = runpy.run_path(script, init_globals=inputs)
    assert output['valid'], output['status']
    assert list(output['joint_plan'].branches.values()) == saved['configurations']
    np.testing.assert_allclose(output['planned_tcp'][0].origin, (1000, 2000, 3000))
    assert output['configurations'][0]['lift'] == .25
    assert len(output['base_planes']) == 2
    # Simulate a recompute with existing output globals and a broken file.
    saved['selected_tcp_rotations'] = []
    write_result(path, saved)
    failed = runpy.run_path(script, init_globals=dict(output, **inputs))
    assert not failed['valid'] and not failed['path_complete']
    assert failed['planned_tcp'] == failed['base_planes'] == failed['configurations'] == []
    assert failed['base_plane'] is failed['joint_plan'] is failed['result'] is None


@pytest.mark.parametrize('scale', [0, -1, float('nan'), float('inf')])
def test_component_rejects_invalid_model_units(saved_run, gh, scale):
    script = str(Path(__file__).resolve().parents[1] / 'examples' / 'grasshopper_import_result.py')
    output = runpy.run_path(script, init_globals=dict(result_path=str(saved_run[0]), units_to_metres=scale))
    assert output['result'] is None and 'units_to_metres' in output['status']

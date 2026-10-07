import json
import sqlite3
from pathlib import Path

import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox import stationary_workflow as workflow
from motion_toolbox.execution import PlanningCancelled
from test_grasshopper_entrypoints import robot_fixture


class Scene:
    def __init__(self, names, accept=None):
        self.joint_names = names
        self.accept = accept or (lambda q, b: True)
        self.calls = []
        self.closed = False
        self.last_failure = 'fixture obstacle'
    def set_fixed_joints(self, values):
        self.fixed = dict(values)
    def is_base_valid(self, base, *, clearance=0.):
        self.calls.append(('base', clearance))
        return True
    def is_valid(self, q, base, *, clearance=0.):
        self.calls.append(('arm', clearance))
        return self.accept(q, base)
    def close(self):
        self.closed = True


def inputs():
    robot, q, targets, names = robot_fixture()
    return dict(robot=robot, target_planes=targets, candidate_planes=[Plane.world_xy()],
                arm_in_base=Plane.world_xy(), rotation_steps=1), q, names


@pytest.mark.parametrize('fast', [True, False])
def test_named_start_and_output_fixed_joints(fast):
    from compas_robots import Configuration
    args, q, names = inputs()
    scene = Scene(names)
    start = Configuration([.2]+list(reversed(q)), [2]+[0]*6, ['lift']+list(reversed(names)))
    result = workflow.plan_stationary_base(**args, current_pose=start, scene=scene,
        fast_validation=fast, fixed_joint_values={'lift': .3})
    assert result['valid'] and result['path_complete']
    assert result['configuration_branch_check_applied']
    assert len(result['selected_configuration_branch']) == 3
    assert scene.fixed['lift'] == .3 and not scene.closed
    for row, config in zip(result['configurations'], result['configuration_objects']):
        assert config['lift'] == .3
        assert config.joint_names == ['lift']+names
        np.testing.assert_allclose([config[n] for n in names], row)
    assert result['effective_settings']['current_pose'] == q


@pytest.mark.parametrize('strategy,fast', [('heuristic', True), ('heuristic', False), ('adaptive', True)])
def test_branch_boundary_start_returns_no_validated_workflow_path(strategy, fast, monkeypatch):
    from motion_toolbox import adaptive_stationary
    monkeypatch.setattr(adaptive_stationary, 'footprint_clearance', lambda world, targets: None)
    args, _, names = inputs()
    result = workflow.plan_stationary_base(**args, current_pose=[0.]*6, scene=Scene(names),
        search_strategy=strategy, fast_validation=fast)
    assert not result['valid'] and not result['fabrication_validated'] and not result['path_complete']
    assert result['configurations'] == result['configuration_objects'] == result['selected_target_planes'] == []
    assert result['configuration_branch_check_applied']
    assert result['edge_rejection_reasons']['ambiguous_configuration_branch'] > 0
    assert 'boundary' in result['initial_state_failure']
    assert 'branch boundaries' in result['status']


@pytest.mark.parametrize('bad', [[float('nan')]*6, [0.]*5, [7.]*6])
def test_bad_start_rejected_before_scene_queries(bad):
    args, _, names = inputs()
    scene = Scene(names)
    with pytest.raises(ValueError):
        workflow.plan_stationary_base(**args, current_pose=bad, scene=scene)
    assert scene.calls == [] and not scene.closed


@pytest.mark.parametrize('fast', [False, True])
def test_start_collision_prevents_path_and_reports_reason(fast):
    args, q, names = inputs()
    scene = Scene(names, lambda values, base: not np.allclose(values, q))
    result = workflow.plan_stationary_base(**args, current_pose=q, scene=scene, fast_validation=fast)
    assert not result['valid'] and not result['path_complete']
    assert result['selected_target_planes'] == []
    assert result['initial_state_failure'] == 'fixture obstacle'
    assert 'Starting configuration collision' in result['status']
    assert len(scene.calls) == 2


def test_collision_disabled_is_never_valid_and_no_path_is_explicit():
    args, _, _ = inputs()
    result = workflow.plan_stationary_base(**args, collision_check=False)
    assert result['path_complete'] and not result['valid']
    assert not result['collision_check_applied']
    result = workflow.plan_stationary_base(**args, build_path=False)
    assert not result['path_complete'] and not result['valid']
    assert 'build_path=False' in result['status'] and result['selected_target_planes'] == []


@pytest.mark.parametrize('fast', [False, True])
@pytest.mark.parametrize('with_start', [False, True])
def test_speed_limits_and_disconnected_diagnostics(fast, with_start):
    args, q, names = inputs()
    result = workflow.plan_stationary_base(**args, scene=Scene(names), fast_validation=fast,
        current_pose=q if with_start else None, time_intervals=[.001]*(3 if with_start else 2),
        max_joint_speed=[.1]*6)
    assert result['found'].base_planes and not result['valid']
    assert not result['path_complete'] and result['speed_checked']
    detail = result['disconnected_detail']
    assert detail['to_target'] == 1 and detail['from_target'] == 0
    assert detail['minimum_joint_step_limit_ratio'] > 1
    assert detail['joint_step_limits'] == pytest.approx([.0001]*6)
    assert len(detail['joint_deltas_at_nearest_pair']) == 6
    assert 'max_joint_speed' in result['status']
    good = workflow.plan_stationary_base(**args, scene=Scene(names), fast_validation=fast,
        current_pose=q if with_start else None, time_intervals=[1.]*(3 if with_start else 2), max_joint_speed=2.5)
    assert good['valid'] and good['disconnected_detail'] is None


@pytest.mark.parametrize('options', [dict(max_joint_speed=1), dict(time_intervals=[1]),
    dict(time_intervals=[0, 1]), dict(time_intervals=[1, 1], max_joint_speed=-1),
    dict(time_intervals=[1, 1], max_joint_speed=[1, 2])])
def test_invalid_speed_inputs(options):
    args, _, _ = inputs()
    with pytest.raises(ValueError):
        workflow.plan_stationary_base(**args, **options)


def test_supplied_scene_clearance_ownership_and_conflicts():
    args, q, names = inputs()
    scene = Scene(names)
    result = workflow.plan_stationary_base(**args, scene=scene, current_pose=q,
        collision_options={'clearance': .015})
    assert result['valid'] and not scene.closed
    assert all(clearance == .015 for kind, clearance in scene.calls)
    assert result['effective_settings']['collision_scene_source'] == 'external'
    for extra in (dict(collision_meshes=[object()]), dict(collision_options={'ground_z': 0}), dict(collision_check=False)):
        with pytest.raises(ValueError):
            workflow.plan_stationary_base(**args, scene=scene, **extra)
    with pytest.raises(ValueError, match='joint order'):
        workflow.plan_stationary_base(**args, scene=Scene(list(reversed(names))))
    assert not scene.closed


@pytest.mark.parametrize('bad', [{'clearance': -1}, {'clearance': float('nan')},
    {'ground_z': float('inf')}, {'support_links': ['root']}, {'typo': 1}])
def test_invalid_collision_options(bad):
    args, _, _ = inputs()
    with pytest.raises(ValueError):
        workflow.plan_stationary_base(**args, collision_options=bad)


def test_owned_scene_options_defaults_ground_and_closure(monkeypatch):
    args, _, _ = inputs()
    original = workflow.PybulletServer
    worlds = []
    class Tracked(original):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.closed = False
            worlds.append(self)
        def close(self):
            self.closed = True
            super().close()
    monkeypatch.setattr(workflow, 'PybulletServer', Tracked)
    options = {'ground_z': -1., 'support_links': ['root'], 'allowed_pairs': [['root', 'link2']]}
    result = workflow.plan_stationary_base(**args, collision_options=options)
    world = worlds[-1]
    assert result['valid'] and world.closed
    assert world.base_collision_model == 'detailed' and not world.exclude_gps
    assert world.environment[0][1] == {world.links['root']}
    assert frozenset(['root', 'link2']) in world.allowed_pairs
    assert result['effective_settings']['collision_options'] == options
    with pytest.raises(ValueError, match='support'):
        workflow.plan_stationary_base(**args, collision_options={'ground_z': -1., 'support_links': ['bad']})
    assert worlds[-1].closed


@pytest.mark.parametrize('stage', ['setup', 'geometry', 'collision_setup', 'placement', 'ik_candidates', 'joint_graph'])
def test_progress_cancels_without_closing_external_scene(stage):
    args, _, names = inputs()
    scene = Scene(names)
    seen = []
    with pytest.raises(PlanningCancelled):
        workflow.plan_stationary_base(**args, scene=scene, progress=lambda m: seen.append(m['stage']),
                                      cancel_check=lambda: stage in seen)
    assert stage in seen and not scene.closed


def test_cancel_owned_scene_closes_and_restores_cpu_policy(monkeypatch):
    args, _, _ = inputs()
    from motion_toolbox import execution
    from test_execution import Controller
    controller = Controller()
    monkeypatch.setattr(execution, '_controller', lambda: controller)
    original = workflow.PybulletServer
    closed = []
    class Tracked(original):
        def close(self):
            closed.append(True)
            super().close()
    monkeypatch.setattr(workflow, 'PybulletServer', Tracked)
    def progress(message):
        if message['stage'] == 'joint_graph':
            raise PlanningCancelled('test')
    with pytest.raises(PlanningCancelled):
        workflow.plan_stationary_base(**args, progress=progress)
    assert closed == [True]
    assert controller.states[-1] == (1, 2, 2)


def test_one_recording_covers_setup_planning_and_loaded_code(tmp_path, monkeypatch):
    monkeypatch.setenv('TOOLBOX_RECORDING', '1')
    monkeypatch.setenv('TOOLBOX_LOG_DIR', str(tmp_path))
    args, _, _ = inputs()
    result = workflow.plan_stationary_base(**args)
    directories = list(tmp_path.iterdir())
    assert len(directories) == 1 and Path(result['research_run']) == directories[0]
    with sqlite3.connect(str(directories[0]/'run.sqlite3')) as db:
        operations = [row[0] for row in db.execute('SELECT operation FROM steps')]
        assert any('PybulletServer.__init__' in op for op in operations)
        assert any('find_stationary_base' in op for op in operations)
        metadata = json.loads(db.execute('SELECT metadata_json FROM run').fetchone()[0])
        assert db.execute('SELECT status FROM run').fetchone()[0] == 'ok'
    from motion_toolbox import __version__
    assert metadata['loaded_modules']['motion_toolbox']['version'] == __version__
    info = result['loaded_modules']['motion_toolbox.stationary_workflow']
    assert len(info['loaded_code_sha256']) == len(info['file_sha256']) == 64
    assert result['effective_settings']['package_version'] == __version__


def test_recording_disabled_does_not_create_run(tmp_path, monkeypatch):
    monkeypatch.setenv('TOOLBOX_RECORDING', '0')
    monkeypatch.setenv('TOOLBOX_LOG_DIR', str(tmp_path))
    args, _, _ = inputs()
    result = workflow.plan_stationary_base(**args, collision_check=False)
    assert result['research_run'] is None and list(tmp_path.iterdir()) == []


def test_existing_recording_is_reused(tmp_path):
    from motion_toolbox.recording import ResearchRun
    args, _, _ = inputs()
    with ResearchRun(tmp_path, name='stationary') as run:
        result = workflow.plan_stationary_base(**args)
        assert Path(result['research_run']) == run.path
    assert len(list(tmp_path.iterdir())) == 1

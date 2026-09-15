"""Run the actual component files, not only the functions they define."""
from pathlib import Path
import runpy
import sys
from types import ModuleType
import numpy as np
import pytest
from compas_robots import RobotModel
from compas_fab.robots import Robot, Tool
from compas.geometry import Frame, Box
from compas.datastructures import Mesh
from motion_toolbox.geometry import Plane
from motion_toolbox.kinematics.ur import forward_kinematics
from motion_toolbox.robot_planning import plan_robot

EXAMPLES = Path(__file__).resolve().parents[1]/'examples'


def robot_fixture():
    links = '<link name="root"/>'
    joints = ''
    for i in range(6):
        links += '<link name="link{}"/>'.format(i)
        parent = 'root' if i == 0 else 'link{}'.format(i-1)
        joints += '<joint name="j{0}" type="revolute"><parent link="{1}"/><child link="link{0}"/><axis xyz="0 0 1"/><limit lower="-3.14159265" upper="3.14159265" velocity="1" effort="1"/></joint>'.format(i, parent)
    links += '<link name="lift"/>'
    joints += '<joint name="lift" type="prismatic"><parent link="root"/><child link="lift"/><axis xyz="0 0 1"/><limit lower="0" upper="1" velocity="1" effort="1"/></joint>'
    robot = Robot(RobotModel.from_urdf_string('<robot name="fixture">'+links+joints+'</robot>'))
    robot.BCF = Frame.worldXY()
    mesh = Mesh.from_shape(Box(.05, .05, .05, frame=Frame((1, 0, 0), (1, 0, 0), (0, 1, 0))))
    robot._attached_tools['arm'] = Tool(mesh, Frame.worldXY(), collision=mesh, connected_to='link5')
    q = [.4, -1.3, 1.1, -.6, .9, .7]
    targets = [forward_kinematics([q[0]+i*.01]+q[1:]) for i in range(3)]
    return robot, q, targets, ['j'+str(i) for i in range(6)]


def test_stationary_component_returns_one_base_and_complete_path(gh):
    robot, q, targets, names = robot_fixture()
    inputs = dict(robot=robot, current_pose=q, target_planes=targets,
        arm_in_base=Plane.world_xy(), rotation_steps=1,
        candidate_planes=[Plane((100,0,0),(1,0,0),(0,1,0)), Plane.world_xy()])
    script = str(EXAMPLES/'grasshopper_stationary_base.py')
    out = runpy.run_path(script, init_globals=inputs)
    assert out['status'].startswith('Found base'), out['status']
    assert np.allclose(out['base_plane'].origin, (0,0,0))
    assert len(out['joint_plan'].branches) == len(targets)
    assert out['candidate_count'] == 2
    assert out['initial_guesses'] == []  # Explicit allowed placements stay authoritative.
    inputs['candidate_planes'] = inputs['candidate_planes'][:1]
    out = runpy.run_path(script, init_globals=inputs)
    assert out['base_plane'] is None
    assert out['status'].startswith('No valid base')


@pytest.fixture
def gh(monkeypatch):
    class Tree:
        @classmethod
        def __class_getitem__(cls, item):
            return cls
        def __init__(self):
            self.branches = {}
        def Add(self, value, path):
            self.branches.setdefault(path, []).append(value)
    module = ModuleType('Grasshopper')
    module.DataTree = Tree
    data = ModuleType('Grasshopper.Kernel.Data')
    data.GH_Path = lambda *x: x
    monkeypatch.setitem(sys.modules, 'Grasshopper', module)
    monkeypatch.setitem(sys.modules, 'Grasshopper.Kernel.Data', data)
    monkeypatch.setattr('motion_toolbox.geometry.to_rhino', lambda p, scale=1: p)


def test_component_executes_with_collision_and_named_output(gh):
    robot, q, targets, names = robot_fixture()
    out = runpy.run_path(str(EXAMPLES/'grasshopper.py'), init_globals=dict(
        robot=robot, current_pose=q, target_planes=targets,
        arm_in_base=Plane.world_xy(), rotation_steps=1,
        fixed_joint_values='{"lift": 0.2}'))
    assert out['status'].startswith('Planned'), out['status']
    assert len(out['joint_plan'].branches) == 3
    assert len(out['base_result']) == 3
    assert out['result']['collision_check_applied']
    assert all(c['lift'] == .2 for c in out['configurations'])
    assert out['timings']['total_seconds'] >= out['timings']['setup_seconds']
    assert out['diagnostics'] == []


@pytest.mark.parametrize('filename', ['grasshopper.py', 'grasshopper_motion_plan.py'])
def test_missing_inputs_clear_outputs(gh, filename):
    failed = runpy.run_path(str(EXAMPLES/filename))
    assert failed['configurations'] == []
    assert failed['result'] is None
    assert 'Error' in failed['status']


def test_minimal_component_tree_and_invalid_rotation(gh):
    _, q, targets, _ = robot_fixture()
    inputs = dict(target_planes=targets, current_pose=q,
                  base_plane=Plane.world_xy(), tcp_plane=Plane.world_xy(), rotation_steps=1)
    out = runpy.run_path(str(EXAMPLES/'grasshopper_motion_plan.py'), init_globals=inputs)
    assert len(out['joint_path'].branches) == 3
    inputs['rotation_steps'] = 0
    out = runpy.run_path(str(EXAMPLES/'grasshopper_motion_plan.py'), init_globals=inputs)
    assert out['status'].startswith('ValueError')


def test_robot_workflow_unit_conversion_and_blocked_environment():
    robot, q, targets, names = robot_fixture()
    millimetres = [Plane(p.origin*1000, p.xaxis, p.yaxis) for p in targets]
    result = plan_robot(robot, millimetres, [], q, Plane.world_xy(), names,
                        model_units_to_metres=.001, collision_check=False, rotation_steps=1)
    assert len(result['configurations']) == 3
    obstacle = Mesh.from_vertices_and_faces([[-2,-2,0], [2,-2,0], [2,2,0], [-2,2,0]], [[0,1,2,3]])
    blocked = plan_robot(robot, targets, [], q, Plane.world_xy(), names,
                         collision_meshes=[obstacle], rotation_steps=1)
    assert blocked['collision_check_applied']
    assert blocked['configurations'] == []
    assert blocked['unreachable_points'] == [0, 1, 2]


def test_configuration_output_is_lift_then_arm_without_wheels():
    robot, q, targets, names = robot_fixture()
    root = robot.model.root
    for index, kind in enumerate((0, 1)):
        link = robot.model.add_link('wheel_link_' + str(index))
        robot.model.add_joint('wheel_joint_' + str(index), kind,
                             root, link, axis=(0, 0, 1))
    robot.model._rebuild_tree()
    result = plan_robot(robot, targets, current_pose=q, arm_in_base=Plane.world_xy(),
                        fixed_joint_values={'lift': .2}, collision_check=False, rotation_steps=1)
    assert len(result['configuration_objects']) == len(targets)
    for config, joints in zip(result['configuration_objects'], result['configurations']):
        assert list(config.joint_names) == ['lift'] + names
        assert list(config.joint_types) == [2] + [0]*6
        np.testing.assert_allclose(config.joint_values, [.2] + joints)


@pytest.mark.parametrize('attached', [True, False])
def test_component_warns_only_when_active_tool_is_missing(gh, monkeypatch, attached):
    from types import SimpleNamespace
    robot, _, targets, _ = robot_fixture()
    if not attached:
        robot._attached_tools.clear()
    messages = []
    kernel = ModuleType('Grasshopper.Kernel')
    kernel.GH_RuntimeMessageLevel = SimpleNamespace(Warning='warning')
    monkeypatch.setitem(sys.modules, 'Grasshopper.Kernel', kernel)
    environment = SimpleNamespace(Component=SimpleNamespace(
        AddRuntimeMessage=lambda level, message: messages.append((level, message))))
    out = runpy.run_path(str(EXAMPLES/'grasshopper.py'), init_globals=dict(
        robot=robot, target_planes=targets, arm_in_base=Plane.world_xy(),
        rotation_steps=1, collision_check=False, ghenv=environment))
    assert out['status'].startswith('Planned'), out['status']
    assert len(out['configurations']) == len(targets)
    assert bool(messages) is not attached
    assert bool(out['result']['warnings']) is not attached
    if not attached:
        assert messages[0][0] == 'warning'
        assert 'No tool is attached' in out['status']
        assert 'no TCP offset' in out['diagnostics'][0]


def test_supplied_scene_is_not_closed():
    from motion_toolbox.collision import PybulletServer
    robot, q, targets, names = robot_fixture()
    with PybulletServer(robot=robot, joint_names=names) as scene:
        result = plan_robot(robot, targets, [], q, Plane.world_xy(), names, scene=scene, rotation_steps=1)
        assert len(result['configurations']) == 3
        assert scene.p.isConnected()


def test_robot_component_optional_mount_and_start_no_sampled_edges(gh, monkeypatch):
    from motion_toolbox.collision import PybulletServer
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    def forbidden(*args, **kwargs):
        raise AssertionError('Default planning must not sample edges')
    monkeypatch.setattr(PybulletServer, 'edge_is_valid', forbidden)
    out = runpy.run_path(str(EXAMPLES/'grasshopper.py'), init_globals=dict(
        robot=robot, target_planes=targets, rotation_steps=1))
    assert out['status'].startswith('Planned'), out['status']
    assert len(out['configurations']) == len(targets)


def test_minimal_component_optional_start(gh):
    _, _, targets, _ = robot_fixture()
    out = runpy.run_path(str(EXAMPLES/'grasshopper_motion_plan.py'), init_globals=dict(
        target_planes=targets, base_plane=Plane.world_xy(), tcp_plane=Plane.world_xy()))
    assert out['status'].startswith('Planned'), out['status']


def test_robot_component_refreshes_cached_required_start_without_source_input(gh, monkeypatch):
    import importlib
    import motion_toolbox.robot_planning as workflow
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    def old_plan(robot, targets, bases, current_pose, arm_in_base, **options):
        raise ValueError('current_pose must contain six finite joint angles in radians')
    monkeypatch.setattr(workflow, 'plan_robot', old_plan)
    inputs = dict(robot=robot, target_planes=targets, rotation_steps=1,
                  collision_check=False)
    script = str(EXAMPLES/'grasshopper.py')
    first = runpy.run_path(script, init_globals=inputs)
    assert first['status'].startswith('Planned'), first['status']
    assert len(first['configurations']) == len(targets)
    def unexpected_reload(module):
        raise AssertionError('Unexpected reload: ' + module.__name__)
    monkeypatch.setattr(importlib, 'reload', unexpected_reload)
    second = runpy.run_path(script, init_globals=inputs)
    assert second['status'].startswith('Planned'), second['status']


def test_offline_tcp_uses_tool_attachment_without_semantics():
    from motion_toolbox.robot_adapter import tcp_from_joints, configuration_from_values
    robot, q, _, names = robot_fixture()
    robot._attached_tools['arm'].frame = Frame((.1, 0, 0), (1, 0, 0), (0, 1, 0))
    config = configuration_from_values(q, names, [0]*6)
    frames = tcp_from_joints(robot, [config])
    np.testing.assert_allclose(frames[0].origin, [.1*np.cos(sum(q)), .1*np.sin(sum(q)), 0], atol=1e-8)


def test_infer_arm_chain_without_tool_and_with_ambiguous_rotary_axis():
    from motion_toolbox.robot_adapter import resolve_arm_joint_names
    robot, _, _, names = robot_fixture()
    assert resolve_arm_joint_names(robot) == names
    tool = robot._attached_tools.pop('arm')
    assert resolve_arm_joint_names(robot) == names
    extra = robot.model.add_link('extra')
    robot.model.add_joint('extra_rotary', 0, robot.model.get_link_by_name('link5'),
                          extra)
    tool.connected_to = 'extra'
    robot._attached_tools['arm'] = tool
    with pytest.raises(ValueError, match='six distinct'):
        resolve_arm_joint_names(robot)
    assert resolve_arm_joint_names(robot, names) == names


def test_joint_name_fallback_without_robot_model():
    from types import SimpleNamespace
    from motion_toolbox.robot_adapter import DEFAULT_ARM_JOINT_NAMES, resolve_arm_joint_names
    assert resolve_arm_joint_names() == list(DEFAULT_ARM_JOINT_NAMES)
    assert resolve_arm_joint_names(SimpleNamespace(model=None)) == list(DEFAULT_ARM_JOINT_NAMES)
    custom = ['custom_'+str(i) for i in range(6)]
    assert resolve_arm_joint_names(names=custom) == custom


def test_stationary_optional_seed_and_mounting_frame(gh):
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    inputs = dict(robot=robot, target_planes=targets,
                  candidate_planes=[Plane.world_xy()], rotation_steps=1)
    script = str(EXAMPLES/'grasshopper_stationary_base.py')
    out = runpy.run_path(script, init_globals=inputs)
    assert out['status'].startswith('Found base'), out['status']
    assert len(out['joint_plan'].branches) == len(targets)
    inputs['current_pose'] = []
    assert out['path_count'] == 'not counted'
    assert out['timings']['path_seconds'] >= 0
    assert len(out['solution_counts']) == len(targets)
    assert runpy.run_path(script, init_globals=inputs)['path_cost'] == out['path_cost']
    robot._RCF = None
    failed = runpy.run_path(script, init_globals=inputs)
    assert failed['base_plane'] is None
    assert 'initialize robot._RCF offline' in failed['status']
    assert failed['status'].startswith(failed['diagnostics'][0])


@pytest.mark.parametrize('scale', [1.0, .001])
def test_stationary_generated_search_uses_constrained_region_in_input_units(gh, scale):
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    inputs = dict(robot=robot, target_planes=[Plane(t.origin/scale, t.xaxis, t.yaxis) for t in targets],
        units_to_metres=scale, grid_spacing=1/scale,
        rotation_steps=1, yaw_steps=1, collision_check=False)
    script = str(EXAMPLES/'grasshopper_stationary_base.py')
    smart = runpy.run_path(script, init_globals=inputs)
    assert smart['initial_guesses'], smart['status']
    assert smart['candidate_count'] > 4
    assert all(np.isfinite(p.origin).all() for p in smart['initial_guesses'])
    assert smart['status'].startswith('Found base'), smart['status']
    assert smart['standoff'] > 0
    assert smart['max_target_distance'] <= 1.75+1e-9
    for base in smart['bases']:
        assert smart['region'].metrics(base)['geometry_valid']
    # Legacy flip/distance/guess switches cannot bypass the geometric rules.
    inputs.update(smart_initial_guess=False, flip_side=True, guess_distance=100, search_margin=100)
    legacy = runpy.run_path(script, init_globals=inputs)
    assert legacy['candidate_count'] == smart['candidate_count']
    assert np.allclose(legacy['base_plane'].origin, smart['base_plane'].origin)


def test_stationary_reports_wrong_side_for_explicit_candidate(gh):
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    target = targets[0]
    wrong_side = Plane(target.origin+target.zaxis, (1,0,0), (0,1,0))
    out = runpy.run_path(str(EXAMPLES/'grasshopper_stationary_base.py'), init_globals=dict(
        robot=robot, target_planes=[target], candidate_planes=[wrong_side], collision_check=False))
    assert out['base_plane'] is None
    assert 'Placement constraints' in out['status']
    assert any('IK checked False' in d for d in out['diagnostics'])


def test_stationary_refreshes_cached_old_adapter(gh, monkeypatch):
    import motion_toolbox.robot_adapter as adapter
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    monkeypatch.setattr(adapter, 'kinematics_from_robot', lambda robot, arm_in_base=None: None)
    out = runpy.run_path(str(EXAMPLES/'grasshopper_stationary_base.py'), init_globals=dict(
        robot=robot, target_planes=targets, candidate_planes=[Plane.world_xy()],
        rotation_steps=1, collision_check=False))
    assert out['status'].startswith('Found base'), out['status']
    assert out['toolbox_loaded_from'].endswith('robot_adapter.py')


def test_stationary_refreshes_old_exporter_with_current_function_signature(gh, monkeypatch):
    import motion_toolbox.robot_adapter as adapter
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    monkeypatch.setattr(adapter, 'STATIONARY_ADAPTER_VERSION', 1)
    def broken_export(*args, **kwargs):
        raise TypeError("cannot pickle 'Mesh' object")
    monkeypatch.setattr(adapter, 'export_robot', broken_export)
    out = runpy.run_path(str(EXAMPLES/'grasshopper_stationary_base.py'), init_globals=dict(
        robot=robot, target_planes=targets, candidate_planes=[Plane.world_xy()], rotation_steps=1))
    assert out['status'].startswith('Found base'), out['status']


def test_stationary_refreshes_cached_overstrict_limit_validator(gh, monkeypatch):
    import motion_toolbox.collision as collision
    monkeypatch.setattr(collision, 'COLLISION_API_VERSION', 2)
    def old_validator(path):
        raise ValueError('lift is missing required limits')
    monkeypatch.setattr(collision, '_validate_urdf', old_validator)
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    out = runpy.run_path(str(EXAMPLES/'grasshopper_stationary_base.py'), init_globals=dict(
        robot=robot, target_planes=targets, candidate_planes=[Plane.world_xy()], rotation_steps=1))
    assert out['status'].startswith('Found base'), out['status']


def test_stationary_refreshes_cached_planner_and_its_imported_dependencies(gh, monkeypatch):
    import motion_toolbox.base_planning as base_planning
    import motion_toolbox.planning as planning
    import motion_toolbox.graph as graph
    def old_base(targets, bases, current_pose=None, **options):
        return planning.calculate_partial_trajectory(current_pose, targets, **options)
    def old_trajectory(current_pose, targets, ik_solver=None):
        return None
    monkeypatch.setattr(base_planning, 'find_stationary_base', old_base)
    monkeypatch.setattr(planning, 'calculate_partial_trajectory', old_trajectory)
    monkeypatch.setattr(base_planning, 'calculate_partial_trajectory', old_trajectory)
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    out = runpy.run_path(str(EXAMPLES/'grasshopper_stationary_base.py'), init_globals=dict(
        robot=robot, target_planes=targets, candidate_planes=[Plane.world_xy()], rotation_steps=1,
        collision_check=False))
    assert out['status'].startswith('Found base'), out['status']
    assert base_planning.calculate_partial_trajectory is planning.calculate_partial_trajectory
    assert base_planning.shortest_path is graph.shortest_path
    assert planning.shortest_path is graph.shortest_path


def test_stationary_does_not_reload_unchanged_pipeline_on_recompute(gh, monkeypatch):
    import importlib
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    inputs = dict(robot=robot, target_planes=targets, candidate_planes=[Plane.world_xy()],
                  rotation_steps=1, collision_check=False)
    script = str(EXAMPLES/'grasshopper_stationary_base.py')
    first = runpy.run_path(script, init_globals=inputs)
    assert first['status'].startswith('Found base'), first['status']
    def unexpected_reload(module):
        raise AssertionError('Unexpected reload: ' + module.__name__)
    monkeypatch.setattr(importlib, 'reload', unexpected_reload)
    second = runpy.run_path(script, init_globals=inputs)
    assert second['status'].startswith('Found base'), second['status']
    assert first['path_count'] == second['path_count']


@pytest.mark.parametrize('build_path', [True, False])
def test_stationary_never_checks_collision_between_configurations(gh, monkeypatch, build_path):
    from motion_toolbox.collision import PybulletServer
    def forbidden(*args, **kwargs):
        raise AssertionError('Transition collision checking must not run')
    monkeypatch.setattr(PybulletServer, 'edge_is_valid', forbidden)
    robot, _, targets, _ = robot_fixture()
    robot._RCF = Frame.worldXY()
    out = runpy.run_path(str(EXAMPLES/'grasshopper_stationary_base.py'), init_globals=dict(
        robot=robot, target_planes=targets, candidate_planes=[Plane.world_xy()],
        rotation_steps=1, build_path=build_path))
    assert out['status'].startswith('Found base'), out['status']
    assert out['path_search_count'] == int(build_path)
    assert int(out['ik_option_count']) > 0


def test_stationary_outdated_installation_reports_loaded_path(gh, monkeypatch):
    import importlib
    import motion_toolbox.robot_adapter as adapter
    monkeypatch.setattr(adapter, 'kinematics_from_robot', lambda robot: None)
    monkeypatch.setattr(importlib, 'reload', lambda module: module)
    out = runpy.run_path(str(EXAMPLES/'grasshopper_stationary_base.py'))
    assert 'Outdated toolbox file:' in out['status']
    assert 'robot_adapter.py' in out['status']
    assert 'toolbox_src' in out['status']
    assert 'restart Rhino' in out['status']

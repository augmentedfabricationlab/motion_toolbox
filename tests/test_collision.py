from types import SimpleNamespace
from pathlib import Path
import numpy as np
import pytest
from motion_toolbox.geometry import Plane
from motion_toolbox.collision import PybulletServer


URDF = '''<robot name="test">
<link name="base"><inertial><origin xyz="0.2 0 0"/><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
<link name="arm"><collision><origin xyz="0.6 0 0"/><geometry><box size="0.4 0.1 0.1"/></geometry></collision></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint>
</robot>'''


@pytest.fixture
def urdf(tmp_path):
    path = tmp_path / 'robot.urdf'
    path.write_text(URDF)
    return path


@pytest.mark.parametrize('clearance', [0.0, .05])
def test_environment_bounds_match_unfiltered_queries(urdf, monkeypatch, clearance):
    monkeypatch.setenv('TOOLBOX_RECORDING', '0')
    with PybulletServer(urdf) as scene:
        for i in range(42):
            scene.add_box([.04]*3, plane=Plane((3+i, 0, 0), (1,0,0), (0,1,0)))
        scene.add_box([.04]*3, plane=Plane((.6, .15, 0), (1,0,0), (0,1,0)))
        overlap = scene._bounds_overlap
        actual_query = scene.p.getClosestPoints
        queries = []
        def counted(*args, **kwargs):
            queries.append(1)
            return actual_query(*args, **kwargs)
        monkeypatch.setattr(scene.p, 'getClosestPoints', counted)
        samples = np.linspace(-1, 1, 31)
        filtered = [(scene.is_valid([q], clearance=clearance), scene.last_failure) for q in samples]
        filtered_count = len(queries)
        queries.clear()
        monkeypatch.setattr(scene, '_bounds_overlap', lambda *args: True)
        full = [(scene.is_valid([q], clearance=clearance), scene.last_failure) for q in samples]
        assert filtered == full
        assert filtered_count < len(queries)/4
        assert any(not valid for valid, _ in full)
        # Current obstacle bounds must be used after the environment moves.
        monkeypatch.setattr(scene, '_bounds_overlap', overlap)
        scene.p.resetBasePositionAndOrientation(scene.environment[0][0], [.6,0,0], [0,0,0,1])
        assert not scene.is_valid([0], clearance=clearance)


def test_missing_urdf_mesh_reports_link_and_filename(tmp_path):
    path = tmp_path / 'missing.urdf'
    path.write_text('<robot name="bad"><link name="wall"><collision><geometry>'
        '<mesh filename="missing.obj"/></geometry></collision></link></robot>')
    with pytest.raises(RuntimeError, match='link wall references a missing or empty mesh') as error:
        PybulletServer(path)
    assert 'missing.obj' in str(error.value)
    assert str(path) in str(error.value)


def test_failed_generated_urdf_is_preserved_after_world_cleanup(tmp_path, monkeypatch):
    import tempfile
    original_mkdtemp = tempfile.mkdtemp
    monkeypatch.setattr(tempfile, 'mkdtemp', lambda suffix=None, prefix=None, dir=None:
        original_mkdtemp(suffix=suffix, prefix=prefix, dir=tmp_path))
    generated = []
    def invalid_export(robot, directory, *args):
        path = directory / 'robot.urdf'
        path.write_text(URDF.replace('<parent link="base"/>', '<parent link="missing"/>'))
        generated.append(path)
        return path, []
    monkeypatch.setattr('motion_toolbox.robot_adapter.export_robot', invalid_export)
    with pytest.raises(RuntimeError, match='invalid parent link') as error:
        PybulletServer(robot=SimpleNamespace(BCF=None))
    retained = Path(str(error.value).split('Inspect retained URDF: ')[1])
    assert retained.is_file()
    assert 'missing' in retained.read_text()
    assert not generated[0].exists()


def test_base_only_checks_exclude_arm_and_ignore_internal_static_assembly_contacts(tmp_path):
    path = tmp_path / 'assembly.urdf'
    path.write_text('''<robot name="assembly">
      <link name="base"><collision><geometry><box size="0.4 0.4 0.4"/></geometry></collision></link>
      <link name="dummy"/><link name="shell"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
      <joint name="a" type="fixed"><parent link="base"/><child link="dummy"/></joint>
      <joint name="b" type="fixed"><parent link="dummy"/><child link="shell"/></joint>
      <link name="arm"><collision><origin xyz="1 0 0"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
      <joint name="hinge" type="continuous"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/></joint>
    </robot>''')
    with PybulletServer(path, joint_names=['hinge']) as original:
        assert not original.is_valid([0])
        assert 'self collision:' in original.last_failure
    with PybulletServer(path, joint_names=['hinge'], check_static_self_collisions=False) as scene:
        assert scene.is_valid([0])
        scene.add_box([.1]*3, plane=Plane((1,0,0),(1,0,0),(0,1,0)))
        assert scene.is_base_valid(Plane.world_xy())  # Obstacle hits only the arm.
        assert scene.p.getJointState(scene.robot, scene.joints[0])[0] == 0
        assert not scene.is_valid([0])
        assert scene.last_failure == 'environment collision: arm / collision_meshes[0]'
        scene.add_box([.1]*3)
        assert not scene.is_base_valid(Plane.world_xy())
    from compas_robots import RobotModel
    from compas_fab.robots import Robot
    robot = Robot(RobotModel.from_urdf_file(str(path)))
    robot.semantics = SimpleNamespace(disabled_collisions={('base','shell')})
    with PybulletServer(robot=robot, joint_names=['hinge']) as scene:
        assert scene.is_valid([0])


def test_tool_may_touch_fixed_wrist_assembly_but_not_upstream_arm_or_environment(tmp_path):
    from compas.datastructures import Mesh
    from compas.geometry import Box, Frame
    path = tmp_path/'wrist.urdf'
    path.write_text('''<robot name="wrist">
      <link name="base"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
      <link name="wrist"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
      <link name="flange"/><link name="tool0"/>
      <joint name="hinge" type="continuous"><origin xyz="1 0 0"/><parent link="base"/><child link="wrist"/><axis xyz="0 0 1"/></joint>
      <joint name="flange_mount" type="fixed"><parent link="wrist"/><child link="flange"/></joint>
      <joint name="tcp" type="fixed"><parent link="flange"/><child link="tool0"/></joint>
    </robot>''')
    mesh = Mesh.from_shape(Box(.15,.15,.15))
    with PybulletServer(path, joint_names=['hinge']) as scene:
        scene.attach_mesh(mesh, 'tool0')
        assert scene.is_valid([0])
        scene.add_box([.1]*3, plane=Plane((1,0,0),(1,0,0),(0,1,0)))
        assert not scene.is_valid([0])
    with PybulletServer(path, joint_names=['hinge']) as scene:
        scene.attach_mesh(mesh, 'tool0', frame=Frame((-1,0,0),(1,0,0),(0,1,0)))
        assert not scene.is_valid([0])
        assert scene.last_failure == 'tool collision: tool[0] / base'


@pytest.mark.parametrize('wrapped', [False, True])
def test_equivalent_joint_turns_reuse_collision_check_but_preserve_configurations(tmp_path, wrapped):
    from types import MethodType
    from motion_toolbox.planning import candidates
    path = tmp_path/'wide_limits.urdf'
    path.write_text(URDF.replace('-3.14', '-7').replace('3.14', '7'))
    class Solver:
        revolute_joints = (0,)
        def __call__(self, target, base):
            return [[.5]]
    with PybulletServer(path, joint_names=['hinge']) as scene:
        actual = scene.is_valid
        calls = []
        def counted(self, q, base, **kwargs):
            calls.append(q)
            return actual(q, base, **kwargs)
        scene.is_valid = MethodType(counted, scene)
        from functools import partial
        check = partial(scene.is_valid, clearance=.01) if wrapped else scene.is_valid
        qs, _, _ = candidates(Plane.world_xy(), Plane.world_xy(), Solver(), [0],
                              check, [[-7,7]])
        assert len(qs) == 3
        assert len(calls) == 1
        assert scene.configuration_cache_key([8]) != scene.configuration_cache_key([8-2*np.pi])


def test_broadphase_self_collisions_match_closest_points_reference(tmp_path):
    path = tmp_path/'chain.urdf'
    path.write_text(URDF.replace('</robot>', '''
      <link name="tip"><collision><origin xyz="0.4 0 0"/>
        <geometry><box size="0.8 0.15 0.15"/></geometry></collision></link>
      <joint name="elbow" type="continuous"><parent link="arm"/><child link="tip"/>
        <origin xyz="0.8 0 0"/><axis xyz="0 0 1"/></joint>
    </robot>'''))
    valid_count = invalid_count = 0
    with PybulletServer(path, joint_names=['hinge','elbow']) as scene:
        for q0 in np.linspace(-3,3,7):
            for q1 in np.linspace(-3,3,9):
                actual = scene.is_valid([q0,q1])
                reference = not any(frozenset((c[3],c[4])) in scene._self_pair_keys
                    for c in scene.p.getClosestPoints(scene.robot, scene.robot, 0))
                assert actual == reference
                valid_count += actual
                invalid_count += not actual
    assert valid_count and invalid_count


def test_isolated_worlds_and_root_inertial_transform(urdf):
    with PybulletServer(urdf) as a, PybulletServer(urdf) as b:
        a.add_box([.05]*3, plane=Plane((.6, 0, 0), (1, 0, 0), (0, 1, 0)))
        assert not a.is_valid([0])
        assert b.is_valid([0])
        assert a.is_valid([1.57])
        assert a.is_valid([0], Plane((2, 0, 0), (1, 0, 0), (0, 1, 0)))
        with pytest.raises(ValueError):
            a.is_valid([0, 1])
        assert not a.is_valid([4])


def test_explicit_obstacle_checks_survive_disabled_global_contacts(urdf):
    with PybulletServer(urdf) as scene:
        obstacle = scene.add_box([.05]*3, plane=Plane((.6, 0, 0), (1,0,0), (0,1,0)))
        assert not scene.is_valid([0])
        assert scene.p.getContactPoints(scene.robot, obstacle) == ()
        assert scene.p.getClosestPoints(scene.robot, obstacle, 0)
        assert scene.last_failure == 'environment collision: arm / collision_meshes[0]'


def test_base_collision_reports_obstacle_and_link_and_clears_failure(urdf):
    with PybulletServer(urdf) as scene:
        scene.add_box([.05]*3)
        assert not scene.is_base_valid(Plane.world_xy())
        assert scene.last_failure == 'base collision: base / collision_meshes[0]'
        assert scene.is_base_valid(Plane((2,0,0), (1,0,0), (0,1,0)))
        assert scene.last_failure is None


def test_tool_collision_and_swept_edge(urdf):
    from compas.datastructures import Mesh
    from compas.geometry import Box, Frame
    mesh = Mesh.from_shape(Box(.2, .2, .2, frame=Frame((1.0, 0, 0), (1, 0, 0), (0, 1, 0))))
    with PybulletServer(urdf) as scene:
        scene.attach_mesh(mesh, 'arm')
        scene.add_box([.05]*3, plane=Plane((1, 0, 0), (1, 0, 0), (0, 1, 0)))
        assert not scene.is_valid([0])
        assert scene.is_valid([-1]) and scene.is_valid([1])
        assert not scene.edge_is_valid([-1], Plane.world_xy(), [1], Plane.world_xy())


def test_real_compas_robot_adapter_preserves_model_and_tool(urdf):
    from compas_robots import RobotModel
    from compas_fab.robots import Robot, Tool
    from compas.datastructures import Mesh
    from compas.geometry import Box, Frame
    model = RobotModel.from_urdf_file(str(urdf))
    robot = Robot(model)
    mesh = Mesh.from_shape(Box(.2, .2, .2, frame=Frame((1, 0, 0), (1, 0, 0), (0, 1, 0))))
    tool = Tool(mesh, Frame((1, 0, 0), (1, 0, 0), (0, 1, 0)), collision=mesh, connected_to='arm')
    robot._attached_tools['main'] = tool
    before = model.to_urdf_string()
    with PybulletServer(robot=robot, joint_names=['hinge']) as scene:
        assert len(scene.tools) == 1
        scene.add_box([.05]*3, plane=Plane((1, 0, 0), (1, 0, 0), (0, 1, 0)))
        assert not scene.is_valid([0])
        assert scene.is_valid([1.57])
    assert model.to_urdf_string() == before
    assert tool.frame.point.x == 1


def test_tilted_stationary_base_and_clearance(urdf):
    from functools import partial
    with PybulletServer(urdf) as scene:
        tilted = Plane((0, 0, 0), (1, 0, 0), (0, 0, 1))
        assert scene.edge_is_valid([0], tilted, [.2], tilted)
        scene.add_box([.05]*3, plane=Plane((.88, 0, 0), (1, 0, 0), (0, 1, 0)))
        base = Plane.world_xy()
        assert scene.is_valid([0], base)
        assert not scene.edge_is_valid([0], base, [0], base, clearance=.1)
        with pytest.raises(ValueError):
            scene.set_fixed_joints({'hinge': 0})

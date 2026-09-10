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

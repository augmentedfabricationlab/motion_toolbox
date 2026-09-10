import math
import numpy as np
from compas.geometry import Frame, Box
from compas.datastructures import Mesh
from compas_robots import RobotModel
from compas_fab.robots import Robot, Tool
from motion_toolbox.robot_adapter import export_robot, kinematics_from_robot, configuration_from_values, tcp_from_joints
from motion_toolbox.geometry import Plane


def test_export_loaded_mesh_and_no_mutation(tmp_path):
    model = RobotModel('mesh_robot')
    mesh = Mesh.from_shape(Box(.1, .1, .1))
    model.add_link('base', collision_meshes=[mesh])
    robot = Robot(model)
    before = model.to_urdf_string()
    path, attachments = export_robot(robot, tmp_path)
    assert path.is_file()
    assert len(list(tmp_path.glob('*.obj'))) == 1
    assert model.to_urdf_string() == before
    assert not attachments


def test_active_tool_tcp_and_explicit_calibration():
    robot = Robot(RobotModel('test'))
    tool = Tool(None, Frame((.1, 0, 0), (1, 0, 0), (0, 1, 0)))
    robot._attached_tools['arm'] = tool
    tool.tool_model.frame = Frame((.2, 0, 0), (1, 0, 0), (0, 1, 0))
    solver = kinematics_from_robot(robot, arm_in_base=Plane.world_xy())
    assert solver.tool.origin[0] == .2


def test_ur_joint_revolutions_respect_limits():
    from motion_toolbox.planning import candidates
    from motion_toolbox.kinematics.ur import forward_kinematics
    from motion_toolbox.kinematics.solver import URKinematics
    q = [.4, -1.3, 1.1, -.6, .9, .7]
    bounds = [[-2*math.pi, 0]] + [[-math.pi, math.pi]]*5
    qs, _, _ = candidates(forward_kinematics(q), Plane.world_xy(), URKinematics(), [0], joint_ranges=bounds)
    assert qs
    assert any(abs(s[0]-(q[0]-2*math.pi)) < 1e-7 for s in qs)

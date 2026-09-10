"""Read the actual MobileRobot/MultiTool classes without writing bytecode there."""
import argparse
import json
from pathlib import Path
from compas_robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas.geometry import Frame, Box
from compas.datastructures import Mesh
from motion_toolbox.collision import PybulletServer
from motion_toolbox.robot_adapter import kinematics_from_robot
from motion_toolbox.geometry import Plane


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.workspace/'mobile_robot_control/src/mobile_robot_control'
    scope = {}
    for filename in ('mobile_robot.py', 'multitool.py'):
        path = root/filename
        exec(compile(path.read_text(encoding='utf-8'), str(path), 'exec'), scope)
    model = RobotModel.from_urdf_string('''<robot name="test"><link name="base"/>
      <link name="flange"/><joint name="joint" type="revolute"><parent link="base"/>
      <child link="flange"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint></robot>''')
    robot = scope['MobileRobot'](model)
    robot._RCF = Frame((0, 0, 1), (1, 0, 0), (0, 1, 0))
    robot.lift_height = .2
    mesh = Mesh.from_shape(Box(.1, .1, .1, frame=Frame((1, 0, 0), (1, 0, 0), (0, 1, 0))))
    frames = {'main': Frame((1, 0, 0), (1, 0, 0), (0, 1, 0)),
              'alternate': Frame((.8, 0, 0), (1, 0, 0), (0, 1, 0))}
    tool = scope['MultiTool'](mesh, frames, collision=mesh, connected_to='flange')
    tool.set_active_tool_frame('alternate')
    robot._attached_tools['arm'] = tool
    solver = kinematics_from_robot(robot, group='arm')
    assert solver.tool.origin[0] == .8
    assert solver.arm_in_base.origin[2] == 1.2
    with PybulletServer(robot=robot, joint_names=['joint']) as scene:
        scene.add_box([.05]*3, plane=Plane((1, 0, 0), (1, 0, 0), (0, 1, 0)))
        assert not scene.is_valid([0])
        assert scene.is_valid([1.57])
    args.output.write_text(json.dumps(dict(result='passed', source_classes=['MobileRobot', 'MultiTool'],
        checked=['URDF export', 'active TCP', 'arm calibration and lift', 'attached mesh collision', 'rotated tool pose']), indent=2))
    print('Actual MobileRobot and MultiTool smoke test passed')


if __name__ == '__main__':
    main()

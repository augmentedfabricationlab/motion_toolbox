import math
import numpy as np
import pytest
from compas.geometry import Frame, Box
from compas.datastructures import Mesh
from compas_robots import RobotModel
from compas_fab.robots import Robot, Tool
from motion_toolbox.robot_adapter import export_robot, kinematics_from_robot, configuration_from_values, tcp_from_joints
from motion_toolbox.geometry import Plane


def mounting_robot():
    links = ['footprint', 'lift_top', 'arm_base_link', 'arm_base'] + ['arm'+str(i) for i in range(6)]
    joints = '''
      <joint name="lift" type="prismatic"><parent link="footprint"/><child link="lift_top"/>
        <origin xyz="0.275 0 1"/><axis xyz="0 0 1"/><limit lower="0" upper="0.7" effort="1" velocity="1"/></joint>
      <joint name="mount" type="fixed"><parent link="lift_top"/><child link="arm_base_link"/></joint>
      <joint name="controller" type="fixed"><parent link="arm_base_link"/><child link="arm_base"/>
        <origin rpy="0 0 3.141592653589793"/></joint>'''
    for i in range(6):
        name = 'arm_shoulder_pan_joint' if i == 0 else 'joint'+str(i)
        parent = 'arm_base_link' if i == 0 else 'arm'+str(i-1)
        joints += '<joint name="{}" type="continuous"><parent link="{}"/><child link="arm{}"/><axis xyz="0 0 1"/></joint>'.format(name, parent, i)
    class OfflineRobot(Robot):
        @property
        def RCF(self):
            raise AssertionError('Must not query live calibration')
    return OfflineRobot(RobotModel.from_urdf_string('<robot name="ur">'+
        ''.join('<link name="{}"/>'.format(n) for n in links)+joints+'</robot>'))


def test_urdf_controller_fallback_includes_rotation_and_lift_once():
    robot = mounting_robot()
    robot.lift_height = .2
    solver = kinematics_from_robot(robot)
    np.testing.assert_allclose(solver.arm_in_base.origin, [.275,0,1.2])
    np.testing.assert_allclose(solver.arm_in_base.xaxis, [-1,0,0], atol=1e-10)
    assert solver.fixed_joint_values == {'lift': .2}
    assert solver.mounting_source == 'URDF controller link arm_base'
    assert getattr(robot, '_RCF', None) is None
    explicit_lift = kinematics_from_robot(robot, fixed_joint_values={'lift':.4})
    np.testing.assert_allclose(explicit_lift.arm_in_base.origin, [.275,0,1.4])
    assert explicit_lift.fixed_joint_values == {'lift':.4}


def test_urdf_fallback_requires_known_upstream_joint_values():
    robot = mounting_robot()
    with pytest.raises(ValueError, match='fixed_joint_values.*lift'):
        kinematics_from_robot(robot)
    with pytest.raises(ValueError, match='Invalid upstream'):
        kinematics_from_robot(robot, fixed_joint_values={'lift':2})


def test_calibration_and_override_take_precedence_over_urdf():
    robot = mounting_robot()
    robot._RCF = Frame((0,0,2), (1,0,0), (0,1,0))
    robot.lift_height = .1
    np.testing.assert_allclose(kinematics_from_robot(robot).arm_in_base.origin, [0,0,2.1])
    np.testing.assert_allclose(kinematics_from_robot(robot, arm_in_base=Plane.world_xy()).arm_in_base.origin, [0,0,0])


def test_robot_planner_infers_lift_without_zero_override():
    from motion_toolbox.robot_planning import plan_robot
    from motion_toolbox.kinematics.ur import forward_kinematics
    robot = mounting_robot()
    robot.lift_height = .2
    solver = kinematics_from_robot(robot)
    q = [.4,-1.3,1.1,-.6,.9,.7]
    target = Plane.from_matrix(solver.arm_in_base.matrix @ forward_kinematics(q).matrix)
    result = plan_robot(robot, [target], [Plane.world_xy()], collision_check=False, rotation_steps=1)
    assert result['configurations']
    assert result['fixed_joint_values']['lift'] == .2
    assert result['configuration_objects'][0]['lift'] == .2
    np.testing.assert_allclose(result['arm_in_base'].origin, [.275,0,1.2])


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


def test_export_native_mesh_without_pickle_or_model_mutation(tmp_path):
    from types import SimpleNamespace
    import xml.etree.ElementTree as ET
    class NativeMesh:
        def __reduce_ex__(self, protocol):
            raise TypeError("cannot pickle 'Mesh' object")
        Vertices = [SimpleNamespace(X=x, Y=y, Z=z) for x,y,z in
                    [(0,0,0), (1,0,0), (1,1,0), (0,1,0)]]
        Faces = [SimpleNamespace(A=0, B=1, C=2, D=3, IsQuad=True)]
    model = RobotModel('native_robot')
    link = model.add_link('base', collision_meshes=[Mesh.from_shape(Box(.1,.1,.1))])
    native = NativeMesh()
    shape = link.collision[0].geometry.shape
    shape.meshes = [native]
    model.native_scene_cache = native
    before = model.to_urdf_string()
    filename = shape.filename
    path, _ = export_robot(Robot(model), tmp_path)
    assert model.to_urdf_string() == before
    assert shape.filename == filename and shape.meshes[0] is native
    mesh_path = ET.parse(path).find('link/collision/geometry/mesh').attrib['filename']
    from pathlib import Path
    obj = Path(mesh_path).read_text()
    assert 'v 1 1 0' in obj
    assert 'f 1 2 3 4' in obj


def test_export_preserves_zero_joint_limits(tmp_path):
    import xml.etree.ElementTree as ET
    robot = mounting_robot()
    before = robot.model.to_urdf_string()
    path, _ = export_robot(robot, tmp_path)
    joint = next(j for j in ET.parse(path).getroot().findall('joint') if j.get('name') == 'lift')
    assert float(joint.find('limit').get('lower')) == 0
    assert float(joint.find('limit').get('upper')) == .7
    assert robot.model.to_urdf_string() == before


def test_large_export_uses_short_xml_lines_and_loads_in_bullet(tmp_path):
    from motion_toolbox.collision import PybulletServer
    model = RobotModel('large_mesh_robot')
    mesh = Mesh.from_shape(Box(.1,.1,.1))
    previous = None
    for i in range(50):
        link = model.add_link('body_'+str(i), collision_meshes=[mesh])
        if previous is not None:
            model.add_joint('fixed_'+str(i), 3, previous, link)
        previous = link
    assert len(model.to_urdf_string()) > 8192
    path, _ = export_robot(Robot(model), tmp_path)
    assert max(map(len, path.read_text().splitlines())) < 1024
    with PybulletServer(path) as scene:
        assert len(scene.links) == 50


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

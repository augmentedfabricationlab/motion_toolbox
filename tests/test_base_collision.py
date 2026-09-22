import xml.etree.ElementTree as ET
import numpy as np
import pytest
from motion_toolbox.collision import PybulletServer
from motion_toolbox.geometry import Plane
from test_collision import URDF


def mobile(path):
    root = ET.Element('robot', name='cover-test')
    ET.SubElement(root, 'link', name='footprint')
    def link(name, parent, xyz, size, kind='fixed', joint=None):
        node = ET.SubElement(root, 'link', name=name)
        if size:
            ET.SubElement(ET.SubElement(ET.SubElement(node,'collision'),'geometry'),'box',size=size)
        j = ET.SubElement(root,'joint',name=joint or name+'_joint',type=kind)
        ET.SubElement(j,'parent',link=parent); ET.SubElement(j,'child',link=name)
        ET.SubElement(j,'origin',xyz=xyz)
        if kind!='fixed':
            ET.SubElement(j,'axis',xyz='0 0 1')
            ET.SubElement(j,'limit',lower='0',upper='1',effort='1',velocity='1')
    link('chassis_link','footprint','0 0 .2','1 .5 .4')
    for name,x,y in [('front_left',.6,.3),('front_right',.6,-.3),('back_left',-.6,.3),('back_right',-.6,-.3)]:
        link(name+'_wheel','footprint',f'{x} {y} .1','.2 .2 .2')
    link('ewellix_lift_base_link','chassis_link','0 0 .5','.2 .2 .5')
    link('ewellix_lift_top_link','ewellix_lift_base_link','0 0 .3','.2 .2 .1','prismatic','lift')
    link('arm_base_link','ewellix_lift_top_link','0 0 .15','.1 .1 .1')
    link('arm','arm_base_link','.4 0 .2','.1 .1 .1','revolute','arm_joint')
    link('lidar','chassis_link','2 0 0','.1 .1 .1')
    ET.ElementTree(root).write(path)
    return path


def test_cover_fills_corners_rotates_and_replaces_accessories_in_final_checks(tmp_path):
    path=mobile(tmp_path/'robot.urdf'); original=path.read_bytes()
    for mode in ('detailed','boxes'):
        with PybulletServer(path,joint_names=['arm_joint'],base_collision_model=mode,
                            check_static_self_collisions=False) as world:
            base=Plane((0,0,0),(0,1,0),(-1,0,0))
            # Local (.65, 0, .3) is beyond the body and between front wheels.
            world.add_box([.01]*3,plane=Plane((0,.65,.3),(1,0,0),(0,1,0)))
            assert world.is_base_valid(base) == (mode=='detailed')
            assert world.is_valid([0],base) == (mode=='detailed')
            if mode=='boxes':
                assert 'lidar' in world.links
                assert world.links['lidar'] not in world.collision_links
                assert set(world.link_names[i] for i in world.collision_links)=={
                    'chassis_link','ewellix_lift_base_link','arm_base_link','arm'}
    assert path.read_bytes()==original


def test_lift_box_matches_configured_extension_and_rejects_stale_bounds(tmp_path):
    path=mobile(tmp_path/'robot.urdf')
    heights=[]
    for value in (0.,.4):
        with PybulletServer(path,joint_names=['arm_joint'],base_collision_model='boxes',
                            fixed_joint_values={'lift':value}) as world:
            box=world.base_collision_geometry['boxes'][1]
            heights.append(box['size'][2])
            world.set_fixed_joints({'lift':value})
            with pytest.raises(ValueError,match='create a new world'):
                world.set_fixed_joints({'lift':value+.1})
    assert heights[1]-heights[0]==pytest.approx(.4,abs=1e-6)


def test_auto_keeps_other_robots_detailed_and_explicit_boxes_require_profile(tmp_path):
    path=tmp_path/'generic.urdf';path.write_text(URDF)
    with PybulletServer(path,base_collision_model='auto') as world:
        assert world.base_collision_geometry['effective']=='detailed'
    with pytest.raises(RuntimeError,match='Cover boxes require'):
        PybulletServer(path,base_collision_model='boxes')


def test_arm_collision_remains_detailed_with_cover_model(tmp_path):
    path=mobile(tmp_path/'robot.urdf')
    with PybulletServer(path,joint_names=['arm_joint'],base_collision_model='boxes',
                        check_static_self_collisions=False) as world:
        world.add_box([.02]*3,plane=Plane((.4,0,1.35),(1,0,0),(0,1,0)))
        assert world.is_base_valid(Plane.world_xy())
        assert not world.is_valid([0],Plane.world_xy())
        assert 'arm' in world.last_failure


def test_root_inertia_does_not_shift_cover_dimensions(tmp_path):
    path=mobile(tmp_path/'robot.urdf')
    with PybulletServer(path,joint_names=['arm_joint'],base_collision_model='boxes') as world:
        expected=world.base_collision_geometry['boxes']
    root=ET.parse(path).getroot()
    inertia=ET.SubElement(root.find('link'),'inertial')
    ET.SubElement(inertia,'origin',xyz='.2 -.1 .4',rpy='0 0 .3')
    ET.SubElement(inertia,'mass',value='1')
    ET.SubElement(inertia,'inertia',ixx='1',iyy='1',izz='1',ixy='0',ixz='0',iyz='0')
    ET.ElementTree(root).write(path)
    with PybulletServer(path,joint_names=['arm_joint'],base_collision_model='boxes') as world:
        for a,b in zip(expected,world.base_collision_geometry['boxes']):
            np.testing.assert_allclose(a['center'],b['center'],atol=1e-6)
            np.testing.assert_allclose(a['size'],b['size'],atol=1e-6)

import math
import numpy as np
from motion_toolbox.geometry import Plane
from motion_toolbox.kinematics.calibrated import CalibratedURKinematics


def chain():
    # Nominal UR20 serial geometry with deliberate non-DH calibration changes.
    origins = [((0,0,.2366),(0,0,0)), ((.0002,0,0),(math.pi/2,0,0)),
               ((-.861,0,0),(0,.001,0)), ((-.7287,0,.201),(0,0,0)),
               ((0,-.1593,0),(math.pi/2,0,0)), ((0,.1543,0),(-math.pi/2,0,0))]
    xml = '<robot name="calibration"><link name="root"/>'
    for i,(xyz,rpy) in enumerate(origins):
        parent = 'root' if i==0 else 'link'+str(i-1)
        xml += '<link name="link{0}"/><joint name="j{0}" type="revolute"><parent link="{1}"/><child link="link{0}"/><origin xyz="{2}" rpy="{3}"/><axis xyz="0 0 1"/><limit lower="-6.283185" upper="6.283185" effort="1" velocity="1"/></joint>'.format(i,parent,' '.join(map(str,xyz)),' '.join(map(str,rpy)))
    xml += '<link name="flange"/><joint name="flange" type="fixed"><parent link="link5"/><child link="flange"/><origin rpy="-1.5707963267948966 -1.5707963267948966 0"/></joint>'
    xml += '<link name="tool0"/><joint name="tcp" type="fixed"><parent link="flange"/><child link="tool0"/><origin rpy="1.5707963267948966 0 1.5707963267948966"/></joint></robot>'
    return xml


def test_refinement_hits_calibrated_target_and_jacobian_agrees_with_differences():
    solver = CalibratedURKinematics(chain(), ['j'+str(i) for i in range(6)],
        controller_link='root',end_link='tool0')
    q = np.array([.4,-1.3,1.1,-.6,.9,.7])
    target,J = solver.forward(q,True)
    for i in range(6):
        shifted = q.copy(); shifted[i] += 1e-6
        np.testing.assert_allclose((solver.forward(shifted)[:3,3]-target[:3,3])/1e-6,
                                   J[:3,i],atol=1e-6)
    rows = solver(Plane.from_matrix(target),Plane.world_xy())
    assert rows
    for row in rows:
        np.testing.assert_allclose(solver.forward(row),target,atol=1e-7)


def test_tcp_and_mount_are_applied_during_refinement():
    mount = Plane((.275,.1,1.2),(0,1,0),(-1,0,0))
    tool = Plane((-.429,.0027,.093),(0,0,1),(0,1,0))
    base = Plane((2,3,0),(0,1,0),(-1,0,0))
    solver = CalibratedURKinematics(chain(), ['j'+str(i) for i in range(6)],
        controller_link='root',end_link='tool0',tool=tool,arm_in_base=mount)
    local = solver.forward([.4,-1.3,1.1,-.6,.9,.7])
    target = Plane.from_matrix(base.matrix@mount.matrix@local@tool.matrix)
    rows = solver(target,base)
    assert rows
    for q in rows:
        np.testing.assert_allclose(base.matrix@mount.matrix@solver.forward(q)@tool.matrix,target.matrix,atol=1e-7)

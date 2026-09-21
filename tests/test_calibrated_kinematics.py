import math
import numpy as np
import pytest
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


@pytest.mark.parametrize('tilted_axes', [False, True])
def test_batched_fk_and_jacobians_match_calibrated_scalar_chain(tilted_axes):
    xml = chain().replace('axis xyz="0 0 1"', 'axis xyz="0.001 -0.002 1"') if tilted_axes else chain()
    solver = CalibratedURKinematics(xml, ['j'+str(i) for i in range(6)],
        controller_link='root', end_link='tool0')
    qs = np.random.default_rng(5).uniform(-6, 6, (40, 6))
    actual, jacobians = solver._forward_batch(qs)
    for q, T, J in zip(qs, actual, jacobians):
        reference, derivative = solver.forward(q, True)
        np.testing.assert_allclose(T, reference, atol=3e-15)
        np.testing.assert_allclose(J, derivative, atol=3e-15)


@pytest.mark.parametrize('tilted_axes', [False, True])
def test_batched_refinement_retains_scalar_branches_and_acceptance(tilted_axes):
    from motion_toolbox.kinematics.ur import inverse_kinematics
    xml = chain().replace('axis xyz="0 0 1"', 'axis xyz="0.001 -0.002 1"') if tilted_axes else chain()
    solver = CalibratedURKinematics(xml, ['j'+str(i) for i in range(6)],
        controller_link='root', end_link='tool0')
    qs = np.random.default_rng(19).uniform(-2, 2, (20, 6))
    qs[:3, 4] = [0., 1e-8, np.pi]  # Wrist singularities / near singularities.
    for q in qs:
        target = solver.forward(q)
        seeds = inverse_kinematics(Plane.from_matrix(target), solver.parameters)
        reference = [row for seed in seeds for row in [solver.refine(target, seed)] if row is not None]
        actual = solver._refine_seeds(target, seeds)
        np.testing.assert_allclose(actual, reference, atol=1e-9, rtol=1e-9)
        for row in actual:
            fk = solver.forward(row)
            assert np.linalg.norm(fk[:3, 3]-target[:3, 3]) <= 1e-7
            assert np.linalg.norm(fk[:3, :3]-target[:3, :3]) <= 1.4e-7


def test_batched_refinement_rejects_unconverged_and_empty_seeds():
    solver = CalibratedURKinematics(chain(), ['j'+str(i) for i in range(6)],
        controller_link='root', end_link='tool0')
    target = np.eye(4)
    target[:3, 3] = [4, 4, 4]
    seeds = np.random.default_rng(12).uniform(-2, 2, (8, 6))
    assert all(solver.refine(target, seed) is None for seed in seeds)
    assert solver._refine_seeds(target, seeds) == []
    assert solver._refine_seeds(target, []) == []

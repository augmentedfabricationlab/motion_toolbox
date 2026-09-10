"""Inverse kinematics solver for UR robots.
This is based on rustr's work, who referenced:
https://github.com/ros-industrial/universal_robot/tree/hydro-devel/ur_kinematics"""

from math import sin, cos, fabs, asin, acos, sqrt, atan2, pi
from ..geometry import Plane


def sign(x):
    """Return the sign of x."""
    if x > 0.0:
        return 1.0
    elif x < 0.0:
        return -1.0
    else:
        return 0.0


ZERO_THRESH = 0.00000001


def inverse_ros(end_effector_pose, params, q6_des=0.0):
    """
    Parameters: end_effector_pose: the 4x4 end effector pose in row-major ordering
                ur_params: UR defined parameters for the model, they are
                different for UR3, UR5 and UR10
                q6_des, an optional parameter which designates what the q6 value
                should take, in case of an infinite solution on that joint.
    Returns:    q_sols, an 8x6 array of doubles returned, 8 possible q joint
                solutions, all angles should be in [0,2 * pi]
    """

    d1, a2, a3, d4, d5, d6 = params

    q_sols = []

    t_02 = -end_effector_pose[0]
    t_00 = end_effector_pose[1]
    t_01 = end_effector_pose[2]
    t_03 = -end_effector_pose[3]
    t_12 = -end_effector_pose[4]
    t_10 = end_effector_pose[5]
    t_11 = end_effector_pose[6]
    t_13 = -end_effector_pose[7]
    t_22 = end_effector_pose[8]
    t_20 = -end_effector_pose[9]
    t_21 = -end_effector_pose[10]
    t_23 = end_effector_pose[11]

    # shoulder rotate joint (q1)
    # q1[2]
    q1 = [0, 0]
    a = d6 * t_12 - t_13
    b = d6 * t_02 - t_03
    r = a * a + b * b
    if fabs(a) < ZERO_THRESH:
        div = 0.0
        if fabs(fabs(d4) - fabs(b)) < ZERO_THRESH:
            div = -sign(d4) * sign(b)
        else:
            div = -d4 / b
        arcsin = asin(div)
        if fabs(arcsin) < ZERO_THRESH:
            arcsin = 0.0
        if arcsin < 0.0:
            q1[0] = arcsin + 2.0 * pi
        else:
            q1[0] = arcsin
        q1[1] = pi - arcsin

    elif fabs(b) < ZERO_THRESH:
        div = 0.0
        if fabs(fabs(d4) - fabs(a)) < ZERO_THRESH:
            div = sign(d4) * sign(a)
        else:
            div = d4 / a
        arccos = acos(div)
        q1[0] = arccos
        q1[1] = 2.0 * pi - arccos

    elif d4 * d4 > r:
        return q_sols
    else:
        arccos = acos(d4 / sqrt(r))
        arctan = atan2(-b, a)
        pos = arccos + arctan
        neg = -arccos + arctan
        if fabs(pos) < ZERO_THRESH:
            pos = 0.0
        if fabs(neg) < ZERO_THRESH:
            neg = 0.0
        if pos >= 0.0:
            q1[0] = pos
        else:
            q1[0] = 2.0 * pi + pos
        if neg >= 0.0:
            q1[1] = neg
        else:
            q1[1] = 2.0 * pi + neg

    # wrist 2 joint (q5)
    q5 = [[0, 0], [0, 0]]
    for i in range(2):
        numer = t_03 * sin(q1[i]) - t_13 * cos(q1[i]) - d4
        div = 0.0
        if fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH:
            div = sign(numer) * sign(d6)
        else:
            div = numer / d6
        arccos = acos(div)
        q5[i][0] = arccos
        q5[i][1] = 2.0 * pi - arccos

    for i in range(2):
        for j in range(2):
            c1 = cos(q1[i])
            s1 = sin(q1[i])
            c5 = cos(q5[i][j])
            s5 = sin(q5[i][j])
            q6 = 0.0

            # wrist 3 joint (q6)
            if fabs(s5) < ZERO_THRESH:
                q6 = q6_des
            else:
                q6 = atan2(
                    sign(s5) * -(t_01 * s1 - t_11 * c1),
                    sign(s5) * (t_00 * s1 - t_10 * c1),
                )
            if fabs(q6) < ZERO_THRESH:
                q6 = 0.0
            if q6 < 0.0:
                q6 += 2.0 * pi

            # RRR joints (q2,q3,q4)
            q2, q3, q4 = [0, 0], [0, 0], [0, 0]

            c6 = cos(q6)
            s6 = sin(q6)
            x04x = -s5 * (t_02 * c1 + t_12 * s1) - c5 * (
                s6 * (t_01 * c1 + t_11 * s1) - c6 * (t_00 * c1 + t_10 * s1)
            )
            x04y = c5 * (t_20 * c6 - t_21 * s6) - t_22 * s5
            p13x = (
                d5 * (s6 * (t_00 * c1 + t_10 * s1) + c6 * (t_01 * c1 + t_11 * s1))
                - d6 * (t_02 * c1 + t_12 * s1)
                + t_03 * c1
                + t_13 * s1
            )
            p13y = t_23 - d1 - d6 * t_22 + d5 * (t_21 * c6 + t_20 * s6)

            c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if fabs(fabs(c3) - 1.0) < ZERO_THRESH:
                c3 = sign(c3)
            elif fabs(c3) > 1.0:
                # TODO NO SOLUTION
                continue

            arccos = acos(c3)
            q3[0] = arccos
            q3[1] = 2.0 * pi - arccos
            denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3
            s3 = sin(arccos)
            a = a2 + a3 * c3
            b = a3 * s3
            q2[0] = atan2((a * p13y - b * p13x) / denom, (a * p13x + b * p13y) / denom)
            q2[1] = atan2((a * p13y + b * p13x) / denom, (a * p13x - b * p13y) / denom)
            c23_0 = cos(q2[0] + q3[0])
            s23_0 = sin(q2[0] + q3[0])
            c23_1 = cos(q2[1] + q3[1])
            s23_1 = sin(q2[1] + q3[1])
            q4[0] = atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0)
            q4[1] = atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1)

            for k in range(2):
                if fabs(q2[k]) < ZERO_THRESH:
                    q2[k] = 0.0
                elif q2[k] < 0.0:
                    q2[k] += 2.0 * pi
                if fabs(q4[k]) < ZERO_THRESH:
                    q4[k] = 0.0
                elif q4[k] < 0.0:
                    q4[k] += 2.0 * pi
                q_sols.append([q1[i], q2[k], q3[k], q4[k], q5[i][j], q6])

    return q_sols


def forward_ros(q, ur_params):
    """
    Parameters: q, the 6 joint angles in radians
                ur_params: UR defined parameters for the model, they are
                different for UR3, UR5 and UR10
    Returns:    T, a list of the 4x4 end effector pose in row-major ordering
    """

    d1, a2, a3, d4, d5, d6 = ur_params

    s1, c1 = sin(q[0]), cos(q[0])
    q234, s2, c2 = q[1], sin(q[1]), cos(q[1])
    s3, c3 = sin(q[2]), cos(q[2])
    q234 += q[2]
    q234 += q[3]
    s5, c5 = sin(q[4]), cos(q[4])
    s6, c6 = sin(q[5]), cos(q[5])
    s234, c234 = sin(q234), cos(q234)

    T = [0.0 for i in range(4 * 4)]

    T[0] = (
        ((c1 * c234 - s1 * s234) * s5) / 2.0
        - c5 * s1
        + ((c1 * c234 + s1 * s234) * s5) / 2.0
    )
    T[1] = (
        c6
        * (
            s1 * s5
            + ((c1 * c234 - s1 * s234) * c5) / 2.0
            + ((c1 * c234 + s1 * s234) * c5) / 2.0
        )
        - (s6 * ((s1 * c234 + c1 * s234) - (s1 * c234 - c1 * s234))) / 2.0
    )
    T[2] = -(c6 * ((s1 * c234 + c1 * s234) - (s1 * c234 - c1 * s234))) / 2.0 - s6 * (
        s1 * s5
        + ((c1 * c234 - s1 * s234) * c5) / 2.0
        + ((c1 * c234 + s1 * s234) * c5) / 2.0
    )
    T[3] = (
        (d5 * (s1 * c234 - c1 * s234)) / 2.0
        - (d5 * (s1 * c234 + c1 * s234)) / 2.0
        - d4 * s1
        + (d6 * (c1 * c234 - s1 * s234) * s5) / 2.0
        + (d6 * (c1 * c234 + s1 * s234) * s5) / 2.0
        - a2 * c1 * c2
        - d6 * c5 * s1
        - a3 * c1 * c2 * c3
        + a3 * c1 * s2 * s3
    )
    T[4] = (
        c1 * c5
        + ((s1 * c234 + c1 * s234) * s5) / 2.0
        + ((s1 * c234 - c1 * s234) * s5) / 2.0
    )
    T[5] = c6 * (
        ((s1 * c234 + c1 * s234) * c5) / 2.0
        - c1 * s5
        + ((s1 * c234 - c1 * s234) * c5) / 2.0
    ) + s6 * ((c1 * c234 - s1 * s234) / 2.0 - (c1 * c234 + s1 * s234) / 2.0)
    T[6] = c6 * ((c1 * c234 - s1 * s234) / 2.0 - (c1 * c234 + s1 * s234) / 2.0) - s6 * (
        ((s1 * c234 + c1 * s234) * c5) / 2.0
        - c1 * s5
        + ((s1 * c234 - c1 * s234) * c5) / 2.0
    )
    T[7] = (
        (d5 * (c1 * c234 - s1 * s234)) / 2.0
        - (d5 * (c1 * c234 + s1 * s234)) / 2.0
        + d4 * c1
        + (d6 * (s1 * c234 + c1 * s234) * s5) / 2.0
        + (d6 * (s1 * c234 - c1 * s234) * s5) / 2.0
        + d6 * c1 * c5
        - a2 * c2 * s1
        - a3 * c2 * c3 * s1
        + a3 * s1 * s2 * s3
    )
    T[8] = (c234 * c5 - s234 * s5) / 2.0 - (c234 * c5 + s234 * s5) / 2.0
    T[9] = (
        (s234 * c6 - c234 * s6) / 2.0 - (s234 * c6 + c234 * s6) / 2.0 - s234 * c5 * c6
    )
    T[10] = (
        s234 * c5 * s6 - (c234 * c6 + s234 * s6) / 2.0 - (c234 * c6 - s234 * s6) / 2.0
    )
    T[11] = (
        d1
        + (d6 * (c234 * c5 - s234 * s5)) / 2.0
        + a3 * (s2 * c3 + c2 * s3)
        + a2 * s2
        - (d6 * (c234 * c5 + s234 * s5)) / 2.0
        - d5 * c234
    )
    T[15] = 1.0
    return T


def inverse_kinematics(
    plane, ur_params=(0.2363, -0.8620, -0.7287, 0.201, 0.1593, 0.1543), q6_des=0.0
):
    """Inverse kinematics function.

    This is the wrapper for the inverse kinematics function from ROS.
    Our robots somehow differ to the standard configuration. Therefore we need
    to swap angles and rotate the first joint by -pi. (The initial position can
    be visualized by loading the meshes.)


    Args:
        the frame to reach.
        ur_params: UR defined parameters for the model. Defaults to UR20.
                    more models can be found in ur_fabrication_control/kinematics/ur_params.py
        q6_des, an optional parameter which designates what the q6 value
        should take, in case of an infinite solution on that joint.

    Returns:
        q_sols, an 8x6 array of doubles returned, 8 possible q joint
        solutions, all angles should be in [0,2 * pi]

    """
    t = [0 for i in range(16)]

    t[0], t[4], t[8] = plane.zaxis
    t[1], t[5], t[9] = plane.xaxis
    t[2], t[6], t[10] = plane.yaxis
    t[3], t[7], t[11] = plane.origin
    t[15] = 1

    try:
        qsols = inverse_ros(t, ur_params, q6_des)
        for i in range(len(qsols)):
            qsols[i][0] -= pi
            qsols[i][5] -= pi
        for i, qsol in enumerate(qsols):
            for j, q in enumerate(qsol):
                if q > pi:
                    q -= 2 * pi
                    # print("rotated -2pi")
                if q < -pi:
                    q += 2 * pi
                    # print("rotated +2pi")
                qsol[j] = q
            qsols[i] = qsol
        return qsols
    except ZeroDivisionError:
        return []


def forward_kinematics(
    configuration, ur_params=(0.2363, -0.8620, -0.7287, 0.201, 0.1593, 0.1543)
):
    """Forward kinematics function.

    This is the wrapper for the forward kinematics function from ROS.
    Our robots somehow differ to the standard configuration. Therefore we need
    to swap angles and rotate the first joint by -pi. (The initial position can
    be visualized by loading the meshes.)

    Args:
        configuration, the 6 joint angles in radians
        ur_params: UR defined parameters for the model. defaults to UR20.
                    more models can be found in ur_fabrication_control/kinematics/ur_params

    Returns:
        the frame
    """

    configuration = list(configuration)
    configuration[0] += pi
    configuration[5] += pi

    T = forward_ros(configuration, ur_params)

    xaxis = [T[1], T[5], T[9]]
    yaxis = [T[2], T[6], T[10]]
    point = [T[3], T[7], T[11]]

    return Plane(point, xaxis, yaxis)

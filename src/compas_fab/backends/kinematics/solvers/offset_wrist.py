from math import sin, cos, fabs, asin, acos, sqrt, atan2, pi
from compas.geometry import Frame
from compas_fab.utilities import sign


def forward_kinematics_offset_wrist(joint_values, params):
    """Forward kinematics function for offset wrist 6-axis robots.

    Parameters
    ----------
    joint_values : list of float
        List of 6 joint values in radians.
    params : list of float
        The offset wrist parameters that specify the robot.

    Returns
    -------
    :class:`compas.geometry.Frame`

    Notes
    -----
    Code adapted from https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
    """
    d1, a2, a3, d4, d5, d6 = params
    q = joint_values

    s1, c1 = sin(q[0]), cos(q[0])
    q23, q234, s2, c2 = q[1], q[1], sin(q[1]), cos(q[1])
    q23 += q[2]
    q234 += q[2]
    s4, c4 = sin(q[3]), cos(q[3])
    q234 += q[3]
    s5, c5 = sin(q[4]), cos(q[4])
    s6, c6 = sin(q[5]), cos(q[5])
    s23, c23 = sin(q23), cos(q23)
    s234, c234 = sin(q234), cos(q234)

    T = [0.0 for _ in range(4 * 4)]

    T[0] = c234 * c1 * s5 - c5 * s1
    T[1] = c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6
    T[2] = -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6
    T[3] = d6 * c234 * c1 * s5 - a3 * c23 * c1 - a2 * c1 * c2 - d6 * c5 * s1 - d5 * s234 * c1 - d4 * s1
    T[4] = c1 * c5 + c234 * s1 * s5
    T[5] = -c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6
    T[6] = s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1
    T[7] = d6 * (c1 * c5 + c234 * s1 * s5) + d4 * c1 - a3 * c23 * s1 - a2 * c2 * s1 - d5 * s234 * s1
    T[8] = -s234 * s5
    T[9] = -c234 * s6 - s234 * c5 * c6
    T[10] = s234 * c5 * s6 - c234 * c6
    T[11] = d1 + a3 * s23 + a2 * s2 - d5 * (c23 * c4 - s23 * s4) - d6 * s5 * (c23 * s4 + s23 * c4)
    T[15] = 1.0

    frame = Frame((T[3], T[7], T[11]), (T[0], T[4], T[8]), (T[1], T[5], T[9]))

    return frame


def inverse_kinematics_offset_wrist(frame, params, q6_des=0.0):
    """Inverse kinematics function for offset wrist 6-axis robots.

    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`
        The frame we want to calculate the inverse kinematics for.
    params : list of float
        The offset wrist parameters that specify the robot.

    Returns
    -------
    list of list

    Notes
    -----
    Code adapted from https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
    """

    ZERO_THRESH = 0.00000001
    d1, a2, a3, d4, d5, d6 = params

    solutions = []

    T02, T12, T22 = frame.xaxis
    T00, T10, T20 = frame.yaxis
    T01, T11, T21 = frame.zaxis
    T03, T13, T23 = frame.point
    T02 *= -1
    T03 *= -1
    T12 *= -1
    T13 *= -1
    T20 *= -1
    T21 *= -1

    # shoulder rotate joint (q1)
    # q1[2]
    q1 = [0, 0]
    A = d6 * T12 - T13
    B = d6 * T02 - T03
    R = A * A + B * B
    if fabs(A) < ZERO_THRESH:
        div = 0.0
        if fabs(fabs(d4) - fabs(B)) < ZERO_THRESH:
            div = -sign(d4) * sign(B)
        else:
            div = -d4 / B
        arcsin = asin(div)
        if fabs(arcsin) < ZERO_THRESH:
            arcsin = 0.0
        if arcsin < 0.0:
            q1[0] = arcsin + 2.0 * pi
        else:
            q1[0] = arcsin
        q1[1] = pi - arcsin

    elif fabs(B) < ZERO_THRESH:
        div = 0.0
        if fabs(fabs(d4) - fabs(A)) < ZERO_THRESH:
            div = sign(d4) * sign(A)
        else:
            div = d4 / A
        arccos = acos(div)
        q1[0] = arccos
        q1[1] = 2.0 * pi - arccos

    elif d4 * d4 > R:
        raise ValueError("No solutions")
    else:
        arccos = acos(d4 / sqrt(R))
        arctan = atan2(-B, A)
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
        numer = T03 * sin(q1[i]) - T13 * cos(q1[i]) - d4
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
                q6 = atan2(sign(s5) * -(T01 * s1 - T11 * c1), sign(s5) * (T00 * s1 - T10 * c1))
            if fabs(q6) < ZERO_THRESH:
                q6 = 0.0
            if q6 < 0.0:
                q6 += 2.0 * pi

            # RRR joints (q2,q3,q4)
            q2, q3, q4 = [0, 0], [0, 0], [0, 0]

            c6 = cos(q6)
            s6 = sin(q6)
            x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1))
            x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5
            p13x = (
                d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1))
                - d6 * (T02 * c1 + T12 * s1)
                + T03 * c1
                + T13 * s1
            )
            p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6)

            c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if fabs(fabs(c3) - 1.0) < ZERO_THRESH:
                c3 = sign(c3)
            elif fabs(c3) > 1.0:
                solutions.extend([None, None])
                continue

            arccos = acos(c3)
            q3[0] = arccos
            q3[1] = 2.0 * pi - arccos
            denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3
            s3 = sin(arccos)
            A = a2 + a3 * c3
            B = a3 * s3
            q2[0] = atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom)
            q2[1] = atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom)
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

                solutions.append([q1[i], q2[k], q3[k], q4[k], q5[i][j], q6])

    return solutions


if __name__ == "__main__":
    from compas.geometry import allclose

    params = [0.089159, -0.42500, -0.39225, 0.10915, 0.09465, 0.0823]  # ur5
    q = [0.2, 5.5, 1.4, 1.3, 2.6, 3.6]
    frame = forward_kinematics_offset_wrist(q, params)
    sol = inverse_kinematics_offset_wrist(frame, params)
    assert allclose(sol[0], q)

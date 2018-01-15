import math
from compas_fab.fab.geometry import Frame
from compas_fab.fab.utilities import sign, argsort
from .ur_kin_ros import forward_ros, inverse_ros


def inverse_kinematics(frame, ur_params, q6_des=0.0):
    """Inverse kinematics function.

    This is the wrapper for the inverse kinematics function from ROS.
    Our robots somehow differ to the standard configuration. Therefore we need
    to swap angles and rotate the first joint by -pi. (The initial position can
    be visualized by loading the meshes.)

    Args:
        the frame to reach.
        ur_params: UR defined parameters for the model
        q6_des, an optional parameter which designates what the q6 value
        should take, in case of an infinite solution on that joint.

    Returns:
        q_sols, an 8x6 array of doubles returned, 8 possible q joint
        solutions, all angles should be in [0,2 * pi]

    """

    T = [0 for i in range(16)]

    T[0], T[4], T[8] = frame.zaxis
    T[1], T[5], T[9] = frame.xaxis
    T[2], T[6], T[10] = frame.yaxis
    T[3], T[7], T[11] = frame.point
    T[15] = 1

    try:
        qsols = inverse_ros(T, ur_params, q6_des)
        for i in range(len(qsols)):
            qsols[i][0] -= math.pi
        return qsols
    except ZeroDivisionError:
        return []


def forward_kinematics(configuration, ur_params):
    """Forward kinematics function.

    This is the wrapper for the forward kinematics function from ROS.
    Our robots somehow differ to the standard configuration. Therefore we need
    to swap angles and rotate the first joint by -pi. (The initial position can
    be visualized by loading the meshes.)

    Args:
        configuration, the 6 joint angles in radians
        ur_params: UR defined parameters for the model

    Returns:
        the frame
    """

    configuration[0] += math.pi

    T = forward_ros(configuration, ur_params)

    xaxis = [T[1], T[5], T[9]]
    yaxis = [T[2], T[6], T[10]]
    point = [T[3], T[7], T[11]]

    return Frame(point, xaxis, yaxis)


if __name__ == "__main__":

    #frame = Frame([56.9907, 410.9482, 432.3825], [0.0000, 1.0000, 0.0000], [1.0000, 0.0000, 0.0000])
    frame = Frame([110.9482, -243.0093, -432.3825], [1.0000, 0.0000, 0.0000], [0.0000, 1.0000, 0.0000])
    ur_params = [89.159, -425.0, -392.25, 109.15, 94.65, 82.3]

    qsols = inverse_kinematics(frame, ur_params)

    for q in qsols:
        print(forward_kinematics(q, ur_params))

    q = [-0.4817717618752444, 2.900620189456401, 4.466606474692679, 3.6283476234151966, 1.5707963267948974, 5.194160742259934]

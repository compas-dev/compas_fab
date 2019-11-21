"""util functions for converting pybullet pose to compas Frame/Transformation
"""

from compas.geometry import Frame, Transformation
from pybullet_planning import Pose
from pybullet_planning import quat_from_euler, quat_from_matrix, euler_from_quat

def pb_pose_from_Frame(frame):
    """ convert compas.Frame to (point, quat).

    Parameters
    ----------
    frame : compas Frame

    Returns
    -------
    tuple
        (point, euler),
        where point: [x, y, z]
              quat:  [roll, pitch, yaw]
    """
    point = [frame.point.x, frame.point.y, frame.point.z]
    euler = frame.euler_angles()
    return (point, quat_from_euler(euler))


def pb_pose_from_Transformation(tf):
    """ convert compas.Transformation to (point, quat).

    Parameters
    ----------
    tf : compas Transformation

    Returns
    -------
    tuple
        (point, euler),
        where point: [x, y, z]
              quat:  [roll, pitch, yaw]
    """
    point = [tf[i, 3] for i in range(3)]
    quat = quat_from_matrix([tf[i, :3] for i in range(3)])
    return (point, quat)


def Frame_from_pb_pose(pose):
    """ convert (point, quat) to compas.Frame

    Parameters
    ----------
    pose : ([x,y,z], [roll, pitch, yaw])

    Returns
    -------
    frame : compas Frame
    """
    frame = Frame.from_euler_angles(euler_from_quat(pose[1]), point=pose[0])
    return frame

def Frame_from_pos_rot(pos, rot):
    """ convert (point, rotation matrix) to compas.Frame

    Parameters
    ----------
    pos : list of float
        [x, y, z]
    rot : list of lists
        row-major 3x3 rotation matrix (same format as the outcome of np.array().tolist())

    Returns
    -------
    frame : compas Frame
    """
    return Frame.from_euler_angles(quat_from_matrix(rot), point=pos)

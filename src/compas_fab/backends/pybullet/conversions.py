from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.geometry import Frame


__all__ = ['pose_from_frame', 'frame_from_pose']


def pose_from_frame(frame):
    """Returns a PyBullet pose from a frame.

    Parameters
    ----------
    frame : :class:`compas.geometry.Frame`

    Returns
    -------
    point, quaternion : tuple
    """
    return list(frame.point), frame.quaternion.xyzw


def frame_from_pose(pose):
    """Returns a frame from a PyBullet pose.

    Parameters
    ----------
    pose : tuple

    Returns
    -------
    :class:`compas.geometry.Frame`
    """
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=point)

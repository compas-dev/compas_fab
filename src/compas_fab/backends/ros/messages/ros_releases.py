from __future__ import absolute_import


class RosDistro(object):
    """Supported/recognized ROS distros"""

    KINETIC = "kinetic"
    MELODIC = "melodic"
    NOETIC = "noetic"

    SUPPORTED_DISTROS = tuple([KINETIC, MELODIC, NOETIC])

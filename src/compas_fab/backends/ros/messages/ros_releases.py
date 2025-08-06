from __future__ import absolute_import


class RosDistro(object):
    """Supported/recognized ROS distros"""

    KINETIC = "kinetic"
    MELODIC = "melodic"
    NOETIC = "noetic"
    HUMBLE = "humble"
    JAZZY = "jazzy"

    SUPPORTED_DISTROS = tuple([KINETIC, MELODIC, NOETIC, HUMBLE, JAZZY])
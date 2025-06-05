from enum import Enum


class RosDistro(Enum):
    """Supported/recognized ROS distros"""

    KINETIC = "kinetic"
    MELODIC = "melodic"
    NOETIC = "noetic"

    SUPPORTED_DISTROS = tuple([KINETIC, MELODIC, NOETIC])

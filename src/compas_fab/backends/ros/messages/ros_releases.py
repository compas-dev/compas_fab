from enum import Enum


class RosDistro(Enum):
    """Supported/recognized ROS distros (ROS 1 and ROS 2)."""

    # ROS 1
    KINETIC = "kinetic"
    MELODIC = "melodic"
    NOETIC = "noetic"

    # ROS 2
    HUMBLE = "humble"
    IRON = "iron"
    JAZZY = "jazzy"
    KILTED = "kilted"
    ROLLING = "rolling"

    SUPPORTED_DISTROS = tuple([KINETIC, MELODIC, NOETIC])

    @property
    def is_ros2(self) -> bool:
        """``True`` if this distro is a ROS 2 distro."""
        return self in (
            RosDistro.HUMBLE,
            RosDistro.IRON,
            RosDistro.JAZZY,
            RosDistro.KILTED,
            RosDistro.ROLLING,
        )

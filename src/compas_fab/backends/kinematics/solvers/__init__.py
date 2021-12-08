from .spherical_wrist_kinematics import *  # noqa: F403
from .offset_wrist_kinematics import *  # noqa: F403


PLANNER_BACKENDS = {
    'ur3': UR3Kinematics,  # noqa: F405
    'ur3e': UR3eKinematics,  # noqa: F405
    'ur5': UR5Kinematics,  # noqa: F405
    'ur5e': UR5eKinematics,  # noqa: F405
    'ur10': UR10Kinematics,  # noqa: F405
    'ur10e': UR10eKinematics,  # noqa: F405
    'staubli_tx260l': Staubli_TX260LKinematics,  # noqa: F405
    'abb_irb4600_40_255': ABB_IRB4600_40_255Kinematics,  # noqa: F405
}

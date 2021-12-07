from .spherical_wrist_kinematics import *  # noqa: F403
from .offset_wrist_kinematics import *  # noqa: F403


IK_SOLVERS = {
    'ur3': UR3Kinematics().inverse,  # noqa: F405
    'ur3e': UR3eKinematics().inverse,  # noqa: F405
    'ur5': UR5Kinematics().inverse,  # noqa: F405
    'ur5e': UR5eKinematics().inverse,  # noqa: F405
    'ur10': UR10Kinematics().inverse,  # noqa: F405
    'ur10e': UR10eKinematics().inverse,  # noqa: F405
    'staubli_tx260l': Staubli_TX260LKinematics().inverse,  # noqa: F405
    'abb_irb4600_40_255': ABB_IRB4600_40_255Kinematics().inverse,  # noqa: F405
}

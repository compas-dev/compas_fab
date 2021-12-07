from .spherical_wrist_kinematics import UR3Kinematics
from .spherical_wrist_kinematics import UR3eKinematics
from .spherical_wrist_kinematics import UR5Kinematics
from .spherical_wrist_kinematics import UR5eKinematics
from .spherical_wrist_kinematics import UR10Kinematics
from .spherical_wrist_kinematics import UR10eKinematics
from .offset_wrist_kinematics import Staubli_TX260LKinematics
from .offset_wrist_kinematics import ABB_IRB4600_40_255Kinematics

IK_SOLVERS = {
    'ur3': UR3Kinematics().inverse,
    'ur3e': UR3eKinematics().inverse,
    'ur5': UR5Kinematics().inverse,
    'ur5e': UR5eKinematics().inverse,
    'ur10': UR10Kinematics().inverse,
    'ur10e': UR10eKinematics().inverse,
    'staubli_tx260l': Staubli_TX260LKinematics().inverse,
    'abb_irb4600_40_255': ABB_IRB4600_40_255Kinematics().inverse,
}
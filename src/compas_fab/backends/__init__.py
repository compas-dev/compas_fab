"""Backend classes for simulation, planning and execution.

Submodules group classes by backend family:

- ROS + MoveIt: [`RosClient`][compas_fab.backends.RosClient],
  [`MoveItPlanner`][compas_fab.backends.MoveItPlanner],
  [`RosFileServerLoader`][compas_fab.backends.RosFileServerLoader],
  [`HttpFileServerLoader`][compas_fab.backends.HttpFileServerLoader].
- PyBullet: [`PyBulletClient`][compas_fab.backends.PyBulletClient],
  [`PyBulletPlanner`][compas_fab.backends.PyBulletPlanner].
- Analytical: [`AnalyticalKinematicsPlanner`][compas_fab.backends.AnalyticalKinematicsPlanner]
  with robot-specific solvers
  (e.g. [`UR10eKinematics`][compas_fab.backends.UR10eKinematics]).

See the [backend architecture guide](../developer/architecture.md) for
details about integrating new backends.
"""

# Base imports
from .exceptions import (
    BackendError,
    BackendFeatureNotSupportedError,
    BackendTargetNotSupportedError,
    TargetModeMismatchError,
    PlanningGroupNotExistsError,
    InverseKinematicsError,
    KinematicsError,
    CollisionCheckError,
    MotionPlanningError,
    MPStartStateInCollisionError,
    MPTargetInCollisionError,
    MPInterpolationInCollisionError,
    MPSearchTimeOutError,
    MPNoIKSolutionError,
    MPNoPlanFoundError,
    MPMaxJumpError,
)

from .tasks import (
    FutureResult,
    CancellableFutureResult,
)

# ROS imports
from .ros import (
    RosClient,
    RosError,
    RosValidationError,
    RosFileServerLoader,
    HttpFileServerLoader,
    MoveItPlanner,
)

# Kinematics imports
from .kinematics import (
    # Kinematics - Analytic IK
    AnalyticalKinematics,
    AnalyticalKinematicsClient,
    AnalyticalInverseKinematics,
    AnalyticalPlanCartesianMotion,
    AnalyticalPyBulletPlanner,
    AnalyticalKinematicsPlanner,
    OffsetWristKinematics,
    SphericalWristKinematics,
    CartesianMotionError,
    # Kinematics - Robot-specific analytic IK
    UR3Kinematics,
    UR3eKinematics,
    UR5Kinematics,
    UR5eKinematics,
    UR10Kinematics,
    UR10eKinematics,
    Staubli_TX260LKinematics,
    ABB_IRB4600_40_255Kinematics,
)

from .pybullet import (
    PyBulletClient,
    PyBulletError,
    PyBulletPlanner,
    AnalyticalPyBulletClient,
    PlanningGroupNotSupported,
)

__all__ = [
    # Exceptions
    "BackendError",
    "BackendFeatureNotSupportedError",
    "BackendTargetNotSupportedError",
    "TargetModeMismatchError",
    "PlanningGroupNotExistsError",
    "InverseKinematicsError",
    "KinematicsError",
    "CollisionCheckError",
    "MotionPlanningError",
    "MPStartStateInCollisionError",
    "MPTargetInCollisionError",
    "MPInterpolationInCollisionError",
    "MPSearchTimeOutError",
    "MPNoIKSolutionError",
    "MPNoPlanFoundError",
    "MPMaxJumpError",
    # Tasks
    "FutureResult",
    "CancellableFutureResult",
    # ROS
    "RosClient",
    "RosError",
    "RosValidationError",
    "RosFileServerLoader",
    "HttpFileServerLoader",
    "MoveItPlanner",
    # Kinematics
    "AnalyticalKinematics",
    "AnalyticalKinematicsClient",
    "AnalyticalInverseKinematics",
    "AnalyticalPlanCartesianMotion",
    "AnalyticalPyBulletPlanner",
    "AnalyticalKinematicsPlanner",
    "OffsetWristKinematics",
    "SphericalWristKinematics",
    "CartesianMotionError",
    # Kinematics - Robot-specific analytic IK
    "UR3Kinematics",
    "UR3eKinematics",
    "UR5Kinematics",
    "UR5eKinematics",
    "UR10Kinematics",
    "UR10eKinematics",
    "Staubli_TX260LKinematics",
    "ABB_IRB4600_40_255Kinematics",
    # PyBullet
    "PyBulletClient",
    "PyBulletError",
    "PyBulletPlanner",
    "AnalyticalPyBulletClient",
    "PlanningGroupNotSupported",
]

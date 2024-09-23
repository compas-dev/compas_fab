from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    "BackendError",
    "BackendFeatureNotSupportedError",
    "BackendTargetNotSupportedError",
    "TargetModeMismatchError",
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
]


class BackendError(Exception):
    """Indicates an exceptional state that caused an error
    within the backend engine."""

    def __init__(self, message):
        super(BackendError, self).__init__(message)
        self.message = message


class BackendFeatureNotSupportedError(Exception):
    """Indicates that the selected backend does not support the selected feature."""

    pass


class BackendTargetNotSupportedError(Exception):
    """Indicates that the selected backend feature does not support the selected target type."""

    pass


class TargetModeMismatchError(Exception):
    """Indicates that the selected TargetMode is not possible with the provided robot cell state.

    For example, if the robot cell state does not have a tool attached to the robot,
    but the TargetMode is set to ``Target.TOOL``.
    """

    pass


# -------------------------
# Kinematics related errors
# -------------------------
class KinematicsError(BackendError):
    """Indicates a kinematic solver exception."""

    def __init__(self, message):
        super(KinematicsError, self).__init__(message)


class InverseKinematicsError(KinematicsError):
    """Indicates that no IK solution could be found by the kinematic solver."""

    def __init__(self, message, target_pcf=None):
        super(InverseKinematicsError, self).__init__("No inverse kinematics solution found.")
        self.message = message
        self.target_pcf = target_pcf


# -------------------------
# Collision checking errors
# -------------------------


class CollisionCheckError(BackendError):
    """Indicates a collision check exception.

    Attributes
    ----------
    message : str
        The error message.
    collision_pairs : list of tuple
        List of pairs of objects that are in collision.
    """

    def __init__(self, message, collision_pairs=None):
        super(CollisionCheckError, self).__init__(message)
        self.message = message
        self.collision_pairs = collision_pairs or []


# -------------------------
# Motion Planning Errors
# -------------------------


class MotionPlanningError(BackendError):
    """Indicates a motion planning exception.

    Attributes
    ----------
    message : str
        The error message.
    """

    def __init__(self, message, partial_trajectory=None):
        super(MotionPlanningError, self).__init__(message)
        self.message = message
        self.partial_trajectory = partial_trajectory


class MPStartStateInCollisionError(MotionPlanningError):
    """Indicates that the start state is in collision.

    Attributes
    ----------
    message : str
        The error message.
    start_state : compas_fab.robots.RobotCellState
        The start state that is in collision.
    collision_pairs : list of tuple
        List of pairs of objects that are in collision.
    """

    def __init__(self, message, start_state=None, collision_pairs=None):
        super(MPStartStateInCollisionError, self).__init__(message)
        self.start_state = start_state
        self.collision_pairs = collision_pairs or []


class MPTargetInCollisionError(MotionPlanningError):
    """Indicates that the target (or one of the waypoints) is in a collision state.

    Attributes
    ----------
    message : str
        The error message.
    target : compas.geometry.Frame
        The target that is in collision.
    collision_pairs : list of tuple
        List of pairs of objects that are in collision.
    """

    def __init__(self, message, target=None, collision_pairs=None):
        super(MPTargetInCollisionError, self).__init__(message)
        self.target = target
        self.collision_pairs = collision_pairs or []


class MPInterpolationInCollisionError(MotionPlanningError):
    """Indicates that the interpolated trajectory is in collision.

    Attributes
    ----------
    message : str
        The error message.
    target : compas.geometry.Frame
        The target that is in collision.
    collision_pairs : list of tuple
        List of pairs of objects that are in collision.
    partial_trajectory : compas_fab.robots.JointTrajectory
        The partial trajectory that was generated before the collision.
    """

    def __init__(self, message, target=None, collision_pairs=None, partial_trajectory=None):
        super(MPInterpolationInCollisionError, self).__init__(message, partial_trajectory)
        self.target = target
        self.collision_pairs = collision_pairs or []


class MPSearchTimeOutError(MotionPlanningError):
    """Indicates that the motion planning search timed out."""

    def __init__(self, message, time_elapsed=None):
        super(MPSearchTimeOutError, self).__init__(message)
        self.time_elapsed = time_elapsed


class MPNoIKSolutionError(MotionPlanningError):
    """Indicates that no IK solution could be found by the motion planner.

    Attributes
    ----------
    message : str
        The error message.
    target : compas.geometry.Frame
        The target that could not be reached.
    """

    def __init__(self, message, target=None, partial_trajectory=None):
        super(MPNoIKSolutionError, self).__init__(message, partial_trajectory)
        self.target = target


class MPNoPlanFoundError(MotionPlanningError):
    """Indicates that no plan could be found by the motion planner."""

    def __init__(self, message, partial_trajectory=None):
        super(MPNoPlanFoundError, self).__init__(message, partial_trajectory)


class MPMaxJumpError(MotionPlanningError):
    """Indicates that the distance between two consecutive JointTrajectoryPoint is too large."""

    def __init__(
        self,
        message=None,
        joint_name=None,
        joint_type=None,
        joint_values_a=None,
        joint_values_b=None,
        value_difference=None,
        value_threshold=None,
    ):
        super(MPMaxJumpError, self).__init__(message)
        self.joint_name = joint_name
        self.joint_type = joint_type
        self.joint_values_a = joint_values_a
        self.joint_values_b = joint_values_b
        self.value_difference = value_difference
        self.value_threshold = value_threshold

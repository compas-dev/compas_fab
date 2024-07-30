from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    "BackendError",
    "BackendFeatureNotSupportedError",
    "InverseKinematicsError",
    "KinematicsError",
    "CollisionCheckInCollisionError",
    "CollisionCheckError",
]


class BackendError(Exception):
    """Indicates an exceptional state that caused an error
    within the backend engine."""

    def __init__(self, message):
        super(BackendError, self).__init__(message)


class BackendFeatureNotSupportedError(Exception):
    """Indicates that the selected backend does not support the selected feature."""

    pass


class BackendTargetNotSupportedError(Exception):
    """Indicates that the selected backend feature does not support the selected target type."""

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

    def __init__(self):
        super(InverseKinematicsError, self).__init__("No inverse kinematics solution found.")


# -------------------------
# Collision checking errors
# -------------------------


class CollisionCheckError(BackendError):
    """Indicates a collision check exception."""

    # TODO: Rewrite this to be a collection of CC errors

    def __init__(self, message):
        super(CollisionCheckError, self).__init__(message)


class CollisionCheckInCollisionError(BackendError):
    """Indicates a collision between two objects is detected during a collision check.

    There are different types of objects that can collide.
    If the backend supports it, the type of object and their names are provided.

    - Type 1: Robot Links - Name: Individual Link Name
    - Type 2. Tool - Name: Tool Name
    - Type 3. Rigidbody - Name: Rigidbody Name

    Attributes
    ----------
    object1_type : int
        Type of the first object.
    object1_name : str
        Name of the first object.
    object2_type : int
        Type of the second object.
    object2_name : str
        Name of the second object.
    """

    # TODO: Rename this class, add the type int as constants and add to a string mapping

    def __init__(self, object1_name, object2_name, object1_type=None, object2_type=None):
        # type(str, str, Optional[int], Optional[int]) -> None
        message = "Collision between '{}' and '{}'".format(object1_name, object2_name)
        super(CollisionCheckInCollisionError, self).__init__(message)
        self.object1_type = object1_type
        self.object1_name = object1_name
        self.object2_type = object2_type
        self.object2_name = object2_name


# class RobotSelfCollisionError(CollisionCheckInCollisionError):
#     """Indicates that the robot is in self-collision."""

#     def __init__(self, message):
#         super(RobotSelfCollisionError, self).__init__(message)
#         self.robot_link_a = None
#         self.robot_link_b = None

# class RobotToolCollisionError(CollisionCheckInCollisionError):
#     """Indicates that the robot is in collision with its tool."""

#     def __init__(self, message):
#         super(RobotToolCollisionError, self).__init__(message)
#         self.robot_link = None
#         self.tool_id = None
#         self.tool_link = None

# class RobotRigidbodyCollisionError(CollisionCheckInCollisionError):
#     """Indicates that the robot is in collision with a rigid body."""

#     def __init__(self, message):
#         super(RobotRigidbodyCollisionError, self).__init__(message)
#         self.robot_link = None
#         self.rigidbody_id = None

# class AttachedStationaryRigidbodyCollisionError(CollisionCheckInCollisionError):
#     """Indicates that an attached rigid body is in collision with a stationary object."""

#     def __init__(self, message):
#         super(AttachedStationaryRigidbodyCollisionError, self).__init__(message)
#         self.rigidbody_id = None
#         self.stationary_id = None

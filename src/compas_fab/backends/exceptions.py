from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    "BackendError",
    "KinematicsError",
    "InverseKinematicsError",
]


class BackendError(Exception):
    """Indicates an exceptional state that caused an error
    within the backend engine."""

    def __init__(self, message):
        super(BackendError, self).__init__(message)


class KinematicsError(BackendError):
    """Indicates a kinematic solver exception."""

    def __init__(self, message):
        super(KinematicsError, self).__init__(message)


class InverseKinematicsError(KinematicsError):
    """Indicates that no IK solution could be found by the kinematic solver."""

    def __init__(self):
        super(InverseKinematicsError, self).__init__("No inverse kinematics solution found.")

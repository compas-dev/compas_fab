from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import BackendError


class KinematicsError(BackendError):
    """Base case for exceptions in ``compas_fab.backends.pybullet``."""
    def __init__(self, message):
        super(KinematicsError, self).__init__(message)


class InverseKinematicsError(KinematicsError):
    """Exception raised when no IK solution can be found in PyBullet."""
    def __init__(self):
        message = "No inverse kinematics solution found."
        super(InverseKinematicsError, self).__init__(message)

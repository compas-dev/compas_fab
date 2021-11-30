from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import BackendError


class KinematicsError(BackendError):
    """Base case for exceptions in ``compas_fab.backends.kinematics``."""

    def __init__(self, message):
        super(KinematicsError, self).__init__(message)


class InverseKinematicsError(KinematicsError):
    """Exception raised when no IK solution can be found in Kinematics."""

    def __init__(self):
        message = "No inverse kinematics solution found."
        super(InverseKinematicsError, self).__init__(message)


class CartesianMotionError(KinematicsError):
    """Exception raised when no path can be found."""

    def __init__(self):
        message = "No complete trajectory found."
        super(InverseKinematicsError, self).__init__(message)

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import BackendError


class PyBulletError(BackendError):
    """Base case for exceptions in ``compas_fab.backends.pybullet``."""
    def __init__(self, message):
        super(PyBulletError, self).__init__(message)


class CollisionError(PyBulletError):
    """Exception raised when two objects have been found to be in collision in PyBullet."""
    def __init__(self, name1, name2):
        message = "Collision between '{}' and '{}'".format(name1, name2)
        super(CollisionError, self).__init__(message)
        self.name1 = name1
        self.name2 = name2


class InverseKinematicsError(PyBulletError):
    """Exception raised when no IK solution can be found in PyBullet."""
    def __init__(self):
        message = "No inverse kinematics solution found."
        super(InverseKinematicsError, self).__init__(message)

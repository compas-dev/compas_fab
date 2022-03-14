from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import KinematicsError


class CartesianMotionError(KinematicsError):
    """Exception raised when no path can be found."""

    def __init__(self):
        message = "No complete trajectory found."
        super(CartesianMotionError, self).__init__(message)

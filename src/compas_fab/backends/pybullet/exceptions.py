from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import BackendError


class PyBulletError(BackendError):
    """Base case for exceptions in ``compas_fab.backends.pybullet``."""

    def __init__(self, message):
        super(PyBulletError, self).__init__(message)

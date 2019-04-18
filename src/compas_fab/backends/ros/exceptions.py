from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.exceptions import BackendError

__all__ = [
    'RosError',
    'RosValidationError',
]


class RosError(BackendError):
    """Wraps an exception that occurred on the communication with ROS."""

    def __init__(self, message, error_code):
        super(RosError, self).__init__('Error code: ' +
                                       str(error_code) +
                                       '; ' + message)
        self.error_code = error_code


class RosValidationError(BackendError):
    """Wraps an exception that occurred on validation of a ROS response."""

    def __init__(self, original_exception, response):
        super(RosValidationError, self).__init__(str(original_exception))
        self.response = response
        self.original_exception = original_exception

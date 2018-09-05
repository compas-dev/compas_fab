from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    'BackendError',
]


class BackendError(Exception):
    """Indicates an exceptional state that caused an error
    within the backend engine."""

    def __init__(self, message):
        super(BackendError, self).__init__(message)

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    'CancellableTask',
]


class CancellableTask(object):
    """Preemptable task represents a long-running operation that can be cancelled."""

    def cancel(self):
        """Attempt to cancel the task.

        If the task is currently being executed and cannot be cancelled
        then the method will return ``False``, otherwise the call will
        be cancelled and the method will return ``True``.
        """
        raise NotImplementedError('Concrete tasks need to provide an implementation')

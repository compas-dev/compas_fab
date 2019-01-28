from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    'CancellableTask',
]


class CancellableTask(object):
    """Preemtable task represents a long-running operation that can be cancelled."""

    def cancel(self):
        raise NotImplementedError('Concrete tasks need to provide an implementation')


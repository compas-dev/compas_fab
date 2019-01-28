from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

__all__ = [
    'CancellableTask',
]


class CancellableTask(object):
    """A tasks that allows to cancel its execution."""

    def cancel(self):
        raise NotImplementedError('Concrete tasks need to provide an implementation')


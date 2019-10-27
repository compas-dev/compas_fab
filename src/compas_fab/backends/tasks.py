from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import threading

__all__ = [
    'FutureResult',
    'CancellableFutureResult'
]


class FutureResult(object):
    """Represents a future result value.

    Futures are the result of asynchronous operations
    but allow to explicitely control when to block and wait
    for its completion."""

    def __init__(self):
        self.done = False
        self.value = None
        self.error = None
        self.event = threading.Event()

    def result(self, timeout=None):
        """Return the feedback value returned by the instruction.

        If the instruction has not yet returned feedback, it will wait
        up to ``timeout`` seconds. If the ``timeout`` expires, the method
        will raise an exception.
        """
        if not self.done:
            if not self.event.wait(timeout):
                raise Exception('Timeout: future result not available')

        if self.error:
            raise self.error

        return self.value

    def _set_result(self, message):
        self.done = True
        self.value = message
        self.error = None
        self.event.set()

    def _set_error_result(self, error):
        """Mark the future as failed.

        Parameters
        ----------
        error : Exception
            An exception instance.
        """
        self.done = True
        self.value = None
        self.error = error
        self.event.set()


class CancellableFutureResult(FutureResult):
    """Represents a future result of a long-running asynchronous operation that can be cancelled."""

    def __init__(self):
        super(CancellableFutureResult, self).__init__()

    def cancel(self):
        """Attempt to cancel the operation.

        If the operation is currently being executed and cannot be cancelled
        then the method will return ``False``, otherwise the call will
        be cancelled and the method will return ``True``.
        """
        raise NotImplementedError('Needs concrete implementation')

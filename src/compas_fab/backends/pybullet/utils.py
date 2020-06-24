from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import logging
import os
import sys
from contextlib import contextmanager
from io import UnsupportedOperation


__all__ = [
    'LOG',
    'redirect_stdout',
]


def get_logger(name):
    logger = logging.getLogger(name)

    try:
        from colorlog import ColoredFormatter
        formatter = ColoredFormatter("%(log_color)s%(levelname)-8s%(reset)s %(white)s%(message)s",
                                     datefmt=None,
                                     reset=True,
                                     log_colors={'DEBUG': 'cyan', 'INFO': 'green',
                                                 'WARNING': 'yellow',
                                                 'ERROR': 'red', 'CRITICAL': 'red',
                                                 }
                                     )
    except ImportError:
        formatter = logging.Formatter('[%(levelname)s] %(message)s')

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    return logger


LOG = get_logger(__name__)


@contextmanager
def redirect_stdout(to=os.devnull, enabled=True):
    '''
    import os

    with stdout_redirected(to=filename):
        print("from Python")
        os.system("echo non-Python applications are also supported")
    '''
    # Pytest interferes with file descriptor capture.
    # Try-except clause exists to disable capture during tests.
    def _redirect_stdout(to_):
        sys.stdout.close()  # + implicit flush()
        os.dup2(to_.fileno(), fd)
        sys.stdout = os.fdopen(fd, 'w')

    try:
        fd = sys.stdout.fileno()
    except UnsupportedOperation:
        enabled = False

    if not enabled:
        yield
    else:
        with os.fdopen(os.dup(fd), 'w') as old_stdout:
            with open(to, 'w') as file:
                _redirect_stdout(to_=file)
            try:
                yield
            finally:
                _redirect_stdout(to_=old_stdout)  # restore stdout. buffering and flags such as CLOEXEC may be different

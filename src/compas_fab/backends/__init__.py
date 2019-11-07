"""
********************************************************************************
compas_fab.backends
********************************************************************************

.. currentmodule:: compas_fab.backends

This package contains classes backends for simulation, planning and execution.

V-REP
-----

.. autosummary::
    :toctree: generated/
    :nosignatures:

    VrepClient

ROS
---

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RosClient
    RosFileServerLoader

Long-running tasks
------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    FutureResult
    CancellableFutureResult

Exceptions
----------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BackendError
    RosError
    RosValidationError
    VrepError

"""

from .exceptions import *               # noqa: F401,F403
from .tasks import *                    # noqa: F401,F403
from .ros.client import *               # noqa: F401,F403
from .ros.exceptions import *           # noqa: F401,F403
from .ros.fileserver_loader import *    # noqa: F401,F403
from .vrep.client import *              # noqa: F401,F403

__all__ = [name for name in dir() if not name.startswith('_')]

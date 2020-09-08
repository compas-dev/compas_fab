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
    VrepPlanner

ROS
---

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RosClient
    RosFileServerLoader
    MoveItPlanner

PyBullet
--------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    PyBulletClient
    PyBulletPlanner

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
    CollisionError
    InverseKinematicsError
    RosError
    RosValidationError
    VrepError


Interfaces
----------

For details about integrating new backends, check
the :ref:`architecture` documentation.

"""

import compas

from .exceptions import *               # noqa: F401,F403
from .tasks import *                    # noqa: F401,F403
from .ros.client import *               # noqa: F401,F403
from .ros.exceptions import *           # noqa: F401,F403
from .ros.fileserver_loader import *    # noqa: F401,F403
from .ros.planner import *              # noqa: F401,F403
from .vrep.client import *              # noqa: F401,F403
from .vrep.helpers import *             # noqa: F401,F403
from .vrep.planner import *             # noqa: F401,F403

if not compas.is_ironpython():
    from .pybullet.client import *            # noqa: F401,F403
    from .pybullet.exceptions import *        # noqa: F401,F403
    from .pybullet.planner import *           # noqa: F401,F403

__all__ = [name for name in dir() if not name.startswith('_')]

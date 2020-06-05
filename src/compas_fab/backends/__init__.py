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
    VrepForwardKinematics
    VrepInverseKinematics
    VrepPlanMotion
    VrepAddAttachedCollisionMesh
    VrepAddCollisionMesh
    VrepRemoveCollisionMesh

ROS
---

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RosClient
    RosFileServerLoader
    MoveItAddAttachedCollisionMesh
    MoveItAddCollisionMesh
    MoveItAppendCollisionMesh
    MoveItForwardKinematics
    MoveItInverseKinematics
    MoveItPlanCartesianMotion
    MoveItPlanMotion
    MoveItPlanningScene
    MoveItRemoveAttachedCollisionMesh
    MoveItRemoveCollisionMesh

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

from .client_manager import *           # noqa: F401,F403
from .exceptions import *               # noqa: F401,F403
from .tasks import *                    # noqa: F401,F403
from .ros.backend_features import *     # noqa: F401,F403
from .ros.client import *               # noqa: F401,F403
from .ros.exceptions import *           # noqa: F401,F403
from .ros.fileserver_loader import *    # noqa: F401,F403
from .vrep.backend_features import *    # noqa: F401,F403
from .vrep.client import *              # noqa: F401,F403
from .vrep.helpers import *             # noqa: F401,F403

__all__ = [name for name in dir() if not name.startswith('_')]

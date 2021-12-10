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
    PyBulletError


Analytical Kinematics
---------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    AnalyticalPyBulletClient
    AnalyticalInverseKinematics
    AnalyticalPlanCartesianMotion


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

from .exceptions import (
    BackendError,
)
from .tasks import (
    FutureResult,
    CancellableFutureResult,
)
from .ros.client import (
    RosClient,
)
from .ros.exceptions import (
    RosError,
    RosValidationError,
)
from .ros.fileserver_loader import (
    RosFileServerLoader,
)
from .ros.planner import (
    MoveItPlanner,
)
from .vrep.client import (
    VrepClient,
)
from .vrep.helpers import (
    VrepError,
)
from .vrep.planner import (
    VrepPlanner,
)

if not compas.IPY:
    from .pybullet.client import (
        PyBulletClient,
    )
    from .pybullet.exceptions import (
        CollisionError,
        InverseKinematicsError,
        PyBulletError,
    )
    from .pybullet.planner import (
        PyBulletPlanner,
    )
    from .kinematics.client import (
        AnalyticalPyBulletClient,
    )
    from .kinematics import (
        AnalyticalInverseKinematics,
    )
    from .kinematics.client import (
        AnalyticalPlanCartesianMotion,
    )

__all__ = [
      'BackendError',
      'CancellableFutureResult',
      'FutureResult',
      'MoveItPlanner',
      'RosClient',
      'RosError',
      'RosFileServerLoader',
      'RosValidationError',
      'VrepClient',
      'VrepError',
      'VrepPlanner',
    ]

if not compas.IPY:
    __all__ += [
        'CollisionError',
        'InverseKinematicsError',
        'PyBulletClient',
        'PyBulletError',
        'PyBulletPlanner',
        'AnalyticalPyBulletClient',
        'AnalyticalInverseKinematics',
        'AnalyticalPlanCartesianMotion',

    ]

"""
********************************************************************************
compas_fab.backends
********************************************************************************

.. currentmodule:: compas_fab.backends

This package contains classes backends for simulation, planning and execution.

ROS
===

Classes to interact with `ROS <https://ros.org/>`_ and the ``MoveIt`` planning
framework.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RosClient
    RosFileServerLoader
    MoveItPlanner
    RosError
    RosValidationError


PyBullet
========

Classes to interact with `PyBullet <http://pybullet.org/>`_.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    PyBulletClient
    PyBulletPlanner
    PyBulletError
    CollisionError

V-REP
=====

Classes to interact with `V-REP <https://www.coppeliarobotics.com/>`_.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    VrepClient
    VrepPlanner
    VrepError


Analytical Kinematics
=====================

Pure-python implementation of analytic IK solvers.

IK solvers
----------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    AnalyticalInverseKinematics
    AnalyticalPlanCartesianMotion
    OffsetWristKinematics
    SphericalWristKinematics
    AnalyticalPyBulletClient

Robot-specific kinematics
-------------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    UR3Kinematics
    UR3eKinematics
    UR5Kinematics
    UR5eKinematics
    UR10Kinematics
    UR10eKinematics
    Staubli_TX260LKinematics
    ABB_IRB4600_40_255Kinematics

Long-running tasks
==================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    FutureResult
    CancellableFutureResult

Exceptions
==========

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BackendError
    CartesianMotionError
    KinematicsError
    InverseKinematicsError

Interfaces
==========

For details about integrating new backends, check
the :ref:`architecture` documentation.

"""

import compas

# Base imports
from .exceptions import (
    BackendError,
    KinematicsError,
    InverseKinematicsError,
)
from .tasks import (
    FutureResult,
    CancellableFutureResult,
)

# ROS imports
from .ros import (
    RosClient,
    RosError,
    RosValidationError,
    RosFileServerLoader,
    MoveItPlanner,
)

# VREP imports
from .vrep import (
    VrepClient,
    VrepError,
    VrepPlanner,
)

# Analytic IK
from .kinematics import (
    AnalyticalInverseKinematics,
    AnalyticalPlanCartesianMotion,
    OffsetWristKinematics,
    SphericalWristKinematics,
    CartesianMotionError,
)

# Robot-specific analytic IK
from .kinematics import (
    UR3Kinematics,
    UR3eKinematics,
    UR5Kinematics,
    UR5eKinematics,
    UR10Kinematics,
    UR10eKinematics,
    Staubli_TX260LKinematics,
    ABB_IRB4600_40_255Kinematics,
)

if not compas.IPY:
    from .pybullet import (
        PyBulletClient,
        CollisionError,
        PyBulletError,
        PyBulletPlanner,
        AnalyticalPyBulletClient,
    )

__all__ = [
    # Base
    'BackendError',
    'KinematicsError',
    'InverseKinematicsError',
    'FutureResult',
    'CancellableFutureResult',
    # ROS
    'RosClient',
    'RosError',
    'RosValidationError',
    'RosFileServerLoader',
    'MoveItPlanner',
    # VREP
    'VrepClient',
    'VrepError',
    'VrepPlanner',
    # Analytic IK
    'AnalyticalInverseKinematics',
    'AnalyticalPlanCartesianMotion',
    'OffsetWristKinematics',
    'SphericalWristKinematics',
    'CartesianMotionError',
    # Robot-specific analytic IK
    'UR3Kinematics',
    'UR3eKinematics',
    'UR5Kinematics',
    'UR5eKinematics',
    'UR10Kinematics',
    'UR10eKinematics',
    'Staubli_TX260LKinematics',
    'ABB_IRB4600_40_255Kinematics',
]

if not compas.IPY:
    __all__ += [
        'PyBulletClient',
        'CollisionError',
        'PyBulletError',
        'PyBulletPlanner',
        'AnalyticalPyBulletClient',
    ]

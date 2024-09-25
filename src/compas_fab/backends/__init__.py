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
    BackendFeatureNotSupportedError
    CollisionCheckError
    CartesianMotionError
    InverseKinematicsError
    KinematicsError

Interfaces
==========

For details about integrating new backends, check
the :ref:`architecture` documentation.

"""

import compas

# Base imports
from .exceptions import (
    BackendError,
    BackendFeatureNotSupportedError,
    BackendTargetNotSupportedError,
    TargetModeMismatchError,
    PlanningGroupNotExistsError,
    InverseKinematicsError,
    KinematicsError,
    CollisionCheckError,
    MotionPlanningError,
    MPStartStateInCollisionError,
    MPTargetInCollisionError,
    MPInterpolationInCollisionError,
    MPSearchTimeOutError,
    MPNoIKSolutionError,
    MPNoPlanFoundError,
    MPMaxJumpError,
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

# Kinematics imports
from .kinematics import (
    # Kinematics - Analytic IK
    AnalyticalInverseKinematics,
    AnalyticalPlanCartesianMotion,
    AnalyticalPyBulletPlanner,
    AnalyticalKinematicsPlanner,
    OffsetWristKinematics,
    SphericalWristKinematics,
    CartesianMotionError,
    # Kinematics - Robot-specific analytic IK
    UR3Kinematics,
    UR3eKinematics,
    UR5Kinematics,
    UR5eKinematics,
    UR10Kinematics,
    UR10eKinematics,
    Staubli_TX260LKinematics,
    ABB_IRB4600_40_255Kinematics,
)

# NOTE: IPY guard because PyBullet do not work in IronPython
# PyBullet imports
if not compas.IPY:
    from .pybullet import (
        PyBulletClient,
        PyBulletError,
        PyBulletPlanner,
        AnalyticalPyBulletClient,
        PlanningGroupNotSupported,
    )

__all__ = [
    # Exceptions
    "BackendError",
    "BackendFeatureNotSupportedError",
    "BackendTargetNotSupportedError",
    "TargetModeMismatchError",
    "PlanningGroupNotExistsError",
    "InverseKinematicsError",
    "KinematicsError",
    "CollisionCheckError",
    "MotionPlanningError",
    "MPStartStateInCollisionError",
    "MPTargetInCollisionError",
    "MPInterpolationInCollisionError",
    "MPSearchTimeOutError",
    "MPNoIKSolutionError",
    "MPNoPlanFoundError",
    "MPMaxJumpError",
    # Tasks
    "FutureResult",
    "CancellableFutureResult",
    # ROS
    "RosClient",
    "RosError",
    "RosValidationError",
    "RosFileServerLoader",
    "MoveItPlanner",
    # Kinematics
    "AnalyticalInverseKinematics",
    "AnalyticalPlanCartesianMotion",
    "AnalyticalPyBulletPlanner",
    "AnalyticalKinematicsPlanner",
    "OffsetWristKinematics",
    "SphericalWristKinematics",
    "CartesianMotionError",
    # Kinematics - Robot-specific analytic IK
    "UR3Kinematics",
    "UR3eKinematics",
    "UR5Kinematics",
    "UR5eKinematics",
    "UR10Kinematics",
    "UR10eKinematics",
    "Staubli_TX260LKinematics",
    "ABB_IRB4600_40_255Kinematics",
]

# PyBullet
if not compas.IPY:
    __all__ += [
        "PyBulletClient",
        "PyBulletError",
        "PyBulletPlanner",
        "AnalyticalPyBulletClient",
        "PlanningGroupNotSupported",
    ]

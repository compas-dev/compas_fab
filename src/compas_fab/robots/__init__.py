"""
********************************************************************************
compas_fab.robots
********************************************************************************

.. currentmodule:: compas_fab.robots

This package contains classes for robot modeling and they are used by the
simulation, planning and execution backends to exchange information.

Main classes
------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Robot
    RobotSemantics
    Configuration
    Tool
    Duration
    Wrench
    Inertia

Path planning
-------------
.. autosummary::
    :toctree: generated/
    :nosignatures:

    JointTrajectory
    JointTrajectoryPoint
    PathPlan

Planning scene
--------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    AttachedCollisionMesh
    CollisionMesh
    PlanningScene

Constraints
-----------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BoundingVolume
    Constraint
    JointConstraint
    OrientationConstraint
    PositionConstraint

Unit conversion
---------------

The unit systems most commonly used in **COMPAS FAB** are **meters** and **radians**.
The following functions help with converting units from one system to the other.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    to_degrees
    to_radians

"""

from .configuration import *          # noqa: F401,F403
from .constraints import *            # noqa: F401,F403
from .path_plan import *              # noqa: F401,F403
from .planning_scene import *         # noqa: F401,F403
from .units import *                  # noqa: F401,F403
from .robot import *                  # noqa: F401,F403
from .semantics import *              # noqa: F401,F403
from .time_ import *                  # noqa: F401,F403
from .tool import *                   # noqa: F401,F403
from .trajectory import *             # noqa: F401,F403
from .wrench import *                 # noqa: F401,F403
from .inertia import *                # noqa: F401,F403

__all__ = [name for name in dir() if not name.startswith('_')]

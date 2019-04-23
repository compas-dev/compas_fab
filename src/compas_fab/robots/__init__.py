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
    Duration

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

from .configuration import *
from .constraints import *
from .path_plan import *
from .planning_scene import *
from .units import *
from .robot import *
from .semantics import *
from .time_ import *
from .trajectory import *

__all__ = [name for name in dir() if not name.startswith('_')]

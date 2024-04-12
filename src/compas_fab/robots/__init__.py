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
    Tool
    Duration
    Wrench
    Inertia
    ReachabilityMap
    DeviationVectorsGenerator
    OrthonormalVectorsFromAxisGenerator

Path planning
-------------
.. autosummary::
    :toctree: generated/
    :nosignatures:

    JointTrajectory
    JointTrajectoryPoint
    Trajectory

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

Built-in robots
---------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RobotLibrary

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

from .constraints import (
    BoundingVolume,
    Constraint,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from .planning_scene import (
    AttachedCollisionMesh,
    CollisionMesh,
    PlanningScene,
)
from .units import (
    to_degrees,
    to_radians,
)
from .reachability_map import (
    ReachabilityMap,
    DeviationVectorsGenerator,
    OrthonormalVectorsFromAxisGenerator,
)
from .robot import (
    Robot,
)
from .robot_library import (
    RobotLibrary,
)
from .semantics import (
    RobotSemantics,
)
from .time_ import (
    Duration,
)
from .tool import (
    Tool,
)
from .trajectory import (
    JointTrajectory,
    JointTrajectoryPoint,
    Trajectory,
)
from .wrench import (
    Wrench,
)
from .inertia import (
    Inertia,
)

__all__ = [
    "AttachedCollisionMesh",
    "BoundingVolume",
    "CollisionMesh",
    "Constraint",
    "Duration",
    "Inertia",
    "JointConstraint",
    "JointTrajectory",
    "JointTrajectoryPoint",
    "OrientationConstraint",
    "PlanningScene",
    "PositionConstraint",
    "ReachabilityMap",
    "DeviationVectorsGenerator",
    "OrthonormalVectorsFromAxisGenerator",
    "Robot",
    "RobotLibrary",
    "RobotSemantics",
    "Tool",
    "Trajectory",
    "Wrench",
    "to_degrees",
    "to_radians",
]

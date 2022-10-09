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
    PathPlan
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

from .configuration import Configuration
from .constraints import (
    BoundingVolume,
    Constraint,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from .path_plan import PathPlan
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
    'AttachedCollisionMesh',
    'BoundingVolume',
    'CollisionMesh',
    'Configuration',
    'Constraint',
    'Duration',
    'Inertia',
    'JointConstraint',
    'JointTrajectory',
    'JointTrajectoryPoint',
    'OrientationConstraint',
    'PathPlan',
    'PlanningScene',
    'PositionConstraint',
    'ReachabilityMap',
    'DeviationVectorsGenerator',
    'OrthonormalVectorsFromAxisGenerator',
    'Robot',
    'RobotSemantics',
    'Tool',
    'Trajectory',
    'Wrench',
    'to_degrees',
    'to_radians',
]

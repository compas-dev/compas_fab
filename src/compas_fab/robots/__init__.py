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

    RobotCell
    RobotCellState
    RigidBody
    RigidBodyState
    RobotSemantics
    ToolState
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


Targets and Waypoints
---------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Target
    FrameTarget
    PointAxisTarget
    ConfigurationTarget
    ConstraintSetTarget
    Waypoints
    FrameWaypoints
    PointAxisWaypoints
    TargetMode

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

Built-in robots and tools
-------------------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ToolLibrary
    RigidBodyLibrary
    RobotCellLibrary

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

from .units import (
    to_degrees,
    to_radians,
)
from .reachability_map import (
    ReachabilityMap,
    DeviationVectorsGenerator,
    OrthonormalVectorsFromAxisGenerator,
)

from .robot_cell import (
    RobotCell,
)
from .state import (
    RigidBodyState,
    RobotCellState,
    ToolState,
)
from .rigid_body import (
    RigidBody,
)

from .robot_library import (
    RigidBodyLibrary,
    RobotCellLibrary,
    ToolLibrary,
)
from .semantics import (
    RobotSemantics,
)
from .targets import (
    ConfigurationTarget,
    ConstraintSetTarget,
    FrameTarget,
    FrameWaypoints,
    PointAxisTarget,
    PointAxisWaypoints,
    Target,
    TargetMode,
    Waypoints,
)
from .time_ import (
    Duration,
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
    # Constraints
    "BoundingVolume",
    "Constraint",
    "JointConstraint",
    "OrientationConstraint",
    "PositionConstraint",
    # Units
    "to_degrees",
    "to_radians",
    # Reachability Map
    "ReachabilityMap",
    "DeviationVectorsGenerator",
    "OrthonormalVectorsFromAxisGenerator",
    # Robot Cell
    "RobotCell",
    # Rigid Body
    "RigidBody",
    # State
    "RigidBodyState",
    "RobotCellState",
    "ToolState",
    # Robot Library
    "ToolLibrary",
    "RigidBodyLibrary",
    "RobotCellLibrary",
    # Semantics
    "RobotSemantics",
    # Targets
    "ConfigurationTarget",
    "ConstraintSetTarget",
    "FrameTarget",
    "FrameWaypoints",
    "PointAxisTarget",
    "PointAxisWaypoints",
    "Target",
    "TargetMode",
    "Waypoints",
    # Time
    "Duration",
    # Trajectory
    "JointTrajectory",
    "JointTrajectoryPoint",
    "Trajectory",
    # Wrench
    "Wrench",
    # Inertia
    "Inertia",
]

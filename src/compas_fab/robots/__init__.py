"""Classes for robot modeling, used by the simulation, planning and
execution backends to exchange information.

The unit systems most commonly used in COMPAS FAB are **meters** and
**radians**; helpers like [`to_degrees`][compas_fab.robots.to_degrees] and
[`to_radians`][compas_fab.robots.to_radians] convert between systems.
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
from .action import (
    Action,
    ActionChain,
)
from .wrench import (
    Wrench,
)
from .inertia import (
    Inertia,
)
from .interpolators import (
    FrameInterpolator,
    PointAxisInterpolator,
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
    # Interpolators
    "FrameInterpolator",
    "PointAxisInterpolator",
    # Action Chain
    "Action",
    "ActionChain",
    # Wrench
    "Wrench",
    # Inertia
    "Inertia",
]

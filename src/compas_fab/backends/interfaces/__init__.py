"""Interfaces required to integrate backends into the simulation, planning
and execution pipeline of COMPAS FAB.

A backend implementation pairs a [`ClientInterface`][compas_fab.backends.interfaces.ClientInterface]
with a [`PlannerInterface`][compas_fab.backends.interfaces.PlannerInterface]
subclass, and overrides planner methods by implementing the relevant
[`BackendFeature`][compas_fab.backends.interfaces.BackendFeature] subclasses
(`CheckCollision`, `ForwardKinematics`, `InverseKinematics`,
`PlanCartesianMotion`, `PlanMotion`, `SetRobotCell`, `SetRobotCellState`).
"""

from .backend_features import BackendFeature
from .backend_features import CheckCollision
from .backend_features import ForwardKinematics
from .backend_features import InverseKinematics
from .backend_features import PlanCartesianMotion
from .backend_features import PlanMotion
from .backend_features import SetRobotCell
from .backend_features import SetRobotCellState
from .client import ClientInterface
from .planner import PlannerInterface

# I believe the following backend features should not be exposed to users
# They should only be used internally by the planner backend
# They are included in the __all__ but not in the `autosummary`

__all__ = [
    "BackendFeature",
    "ClientInterface",
    "CheckCollision",
    "ForwardKinematics",
    "InverseKinematics",
    "PlanCartesianMotion",
    "PlanMotion",
    "PlannerInterface",
    "SetRobotCell",
    "SetRobotCellState",
]

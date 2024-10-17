"""
.. currentmodule:: compas_fab.backends.interfaces

This package defines the interfaces required to integrate backends into
the simulation, planning and execution pipeline of COMPAS FAB.

Client interfaces
=================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ClientInterface
    PlannerInterface

RobotCell interfaces
====================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    SetRobotCell
    SetRobotCellState

Planner feature interfaces
==========================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BackendFeature
    CheckCollision
    ForwardKinematics
    InverseKinematics
    PlanCartesianMotion
    PlanMotion

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

from .backend_features import GetPlanningScene
from .backend_features import ResetPlanningScene

__all__ = [
    "BackendFeature",
    "ClientInterface",
    "CheckCollision",
    "ForwardKinematics",
    "GetPlanningScene",
    "InverseKinematics",
    "PlanCartesianMotion",
    "PlanMotion",
    "PlannerInterface",
    "ResetPlanningScene",
    "SetRobotCell",
    "SetRobotCellState",
]

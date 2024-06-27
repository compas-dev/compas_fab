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

Feature interfaces
==================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BackendFeature
    ForwardKinematics
    InverseKinematics
    PlanMotion
    PlanCartesianMotion

Planning scene interfaces
=========================

.. autosummary::
    :toctree: generated/
    :nosignatures:

    GetPlanningScene
    AddCollisionMesh
    AppendCollisionMesh
    RemoveCollisionMesh
    AddAttachedCollisionMesh
    RemoveAttachedCollisionMesh
    ResetPlanningScene

"""

from .backend_features import AddAttachedCollisionMesh
from .backend_features import AddCollisionMesh
from .backend_features import AppendCollisionMesh
from .backend_features import BackendFeature
from .backend_features import ForwardKinematics
from .backend_features import GetPlanningScene
from .backend_features import InverseKinematics
from .backend_features import PlanCartesianMotion
from .backend_features import PlanMotion
from .backend_features import RemoveAttachedCollisionMesh
from .backend_features import RemoveCollisionMesh
from .backend_features import ResetPlanningScene
from .backend_features import SetRobotCell
from .backend_features import SetRobotCellState
from .client import ClientInterface
from .client import PlannerInterface

__all__ = [
    "AddAttachedCollisionMesh",
    "AddCollisionMesh",
    "AppendCollisionMesh",
    "BackendFeature",
    "ClientInterface",
    "ForwardKinematics",
    "GetPlanningScene",
    "InverseKinematics",
    "PlanCartesianMotion",
    "PlanMotion",
    "PlannerInterface",
    "RemoveCollisionMesh",
    "RemoveAttachedCollisionMesh",
    "ResetPlanningScene",
    "SetRobotCell",
    "SetRobotCellState",
]

"""
********************************************************************************
compas_fab.backends.interfaces
********************************************************************************

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

"""

from .backend_features import AddAttachedCollisionMesh
from .backend_features import AddCollisionMesh
from .backend_features import AppendCollisionMesh
from .backend_features import ForwardKinematics
from .backend_features import GetPlanningScene
from .backend_features import InverseKinematics
from .backend_features import PlanCartesianMotion
from .backend_features import PlanMotion
from .backend_features import RemoveAttachedCollisionMesh
from .backend_features import RemoveCollisionMesh
from .client import ClientInterface
from .client import PlannerInterface

__all__ = [
    'ForwardKinematics',
    'InverseKinematics',
    'PlanMotion',
    'PlanCartesianMotion',
    'GetPlanningScene',
    'AddCollisionMesh',
    'AppendCollisionMesh',
    'RemoveCollisionMesh',
    'AddAttachedCollisionMesh',
    'RemoveAttachedCollisionMesh',
    'ClientInterface',
    'PlannerInterface',
]

"""
********************************************************************************
compas_fab.backends.interfaces
********************************************************************************

.. currentmodule:: compas_fab.backends.interfaces

This package contains classes backends for simulation, planning and execution.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ClientInterface
    PlannerInterface
    ForwardKinematics
    InverseKinematics
    PlanMotion
    PlanCartesianMotion
    GetPlanningScene
    AddCollisionMesh
    AppendCollisionMesh
    RemoveCollisionMesh
    AddAttachedCollisionMesh
    RemoveAttachedCollisionMesh
"""

from .backend_features import *
from .client import *

__all__ = [name for name in dir() if not name.startswith('_')]

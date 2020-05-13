"""
********************************************************************************
compas_fab.backends.vrep
********************************************************************************

.. module:: compas_fab.backends.vrep

Package with functionality to run simulations using the robotic simulation tool
`V-REP <http://www.coppeliarobotics.com/>`_.

.. autosummary::
    :toctree: generated/

    VrepClient
    VrepError
    VrepForwardKinematics
    VrepInverseKinematics
    VrepPlanMotion
    VrepAddAttachedCollisionMesh

"""

from .helpers import VrepError
from .client import VrepClient
from .backend_features import VrepInverseKinematics
from .backend_features import VrepForwardKinematics
from .backend_features import VrepPlanMotion
from .backend_features import VrepAddAttachedCollisionMesh

__all__ = [
    'VrepClient',
    'VrepError',
    'VrepForwardKinematics',
    'VrepInverseKinematics',
    'VrepPlanMotion',
    'VrepAddAttachedCollisionMesh',
]

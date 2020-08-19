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
    VrepPlanner

"""
from __future__ import absolute_import

from .helpers import VrepError
from .client import VrepClient
from .planner import VrepPlanner

__all__ = [
    'VrepClient',
    'VrepError',
    'VrepPlanner',
]

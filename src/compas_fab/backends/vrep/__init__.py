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

"""

from .client import VrepError
from .client import VrepClient

__all__ = [
    'VrepError',
    'VrepClient'
]

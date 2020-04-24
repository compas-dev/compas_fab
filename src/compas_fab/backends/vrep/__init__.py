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

from .helpers import VrepError
from .client import VrepClient

__all__ = [
    'VrepClient',
    'VrepError',
]

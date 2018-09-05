"""
********************************************************************************
compas_fab.backends
********************************************************************************

.. currentmodule:: compas_fab.backends

This package contains classes backends for simulation, planning and execution.

Vrep
--------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    VrepClient

ROS
-------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RosClient

"""

from .vrep import Simulator as VrepClient
from .ros.client import Client as RosClient

"""
********************************************************************************
compas_fab.backends
********************************************************************************

.. currentmodule:: compas_fab.backends

This package contains classes backends for simulation, planning and execution.

V-REP
-----

.. autosummary::
    :toctree: generated/
    :nosignatures:

    VrepClient

ROS
---

.. autosummary::
    :toctree: generated/
    :nosignatures:

    RosClient

Exceptions
----------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BackendError

"""

from .vrep.client import VrepClient
from .ros.client import Client as RosClient

from .exceptions import *
from .exceptions import __all__ as a

__all__ = ['VrepClient', 'RosClient'] + a

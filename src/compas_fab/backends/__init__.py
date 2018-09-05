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
    VrepError

"""

from .exceptions import *
from .vrep.client import *
from .ros.client import Client as RosClient

from .exceptions import __all__ as a
from .vrep.client import __all__ as b

__all__ = a + b + ['RosClient']

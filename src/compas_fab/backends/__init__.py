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

Common
------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    CancellableTask

Exceptions
----------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BackendError
    RosError
    VrepError

"""

from .shared import *
from .exceptions import *
from .ros.client import *
from .vrep.client import *

from .shared import __all__ as a
from .exceptions import __all__ as b
from .ros.client import __all__ as c
from .vrep.client import __all__ as d

__all__ = a + b + c + d

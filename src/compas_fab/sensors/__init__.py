"""
********************************************************************************
compas_fab.sensors
********************************************************************************

.. currentmodule:: compas_fab.sensors

Package containing a common interface to integrate sensors of various vendors.

Main classes
------------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    SerialSensor

Baumer sensors
--------------

.. currentmodule:: compas_fab.sensors.baumer

:mod:`compas_fab.sensors.baumer`

.. autosummary::
    :toctree: generated/
    :nosignatures:

    PosCon3D
    PosConCM

"""

from .base import *
from .baumer import *

__all__ = [name for name in dir() if not name.startswith('_')]

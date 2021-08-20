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

from .base import (
    SerialSensor,
)
from .baumer import (
    PosCon3D,
    PosConCM,
)

__all__ = [
    "PosCon3D",
    "PosConCM",
    "SerialSensor",
]

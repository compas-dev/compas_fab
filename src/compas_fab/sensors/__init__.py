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

.. autosummary::
    :toctree: generated/
    :nosignatures:

    PosCon3D
    PosConCM

Exceptions
----------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    ProtocolError
    SensorTimeoutError

"""

from .base import (
    SerialSensor,
)
from .baumer import (
    PosCon3D,
    PosConCM,
)
from .exceptions import ProtocolError, SensorTimeoutError

__all__ = [
    # base
    'SerialSensor',
    # baumer
    'PosCon3D',
    'PosConCM',
    # exceptions
    'ProtocolError',
    'SensorTimeoutError',
]

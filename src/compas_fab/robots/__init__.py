"""
********************************************************************************
compas_fab.robots
********************************************************************************

.. currentmodule:: compas_fab.robots

This package contains classes for robot modeling and they are used by the
simulation, planning and execution backends to exchange information.

Classes
--------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Configuration
    PathPlan
    Robot
    RobotSemantics
    UrdfImporter

Unit conversion
---------------

The unit systems most commonly used in **compas_fab** are **meters** and **radians**.
The following functions help with converting units from one system to the other.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    to_degrees
    to_radians

"""

from .configuration import *
from .path_plan import *
from .units import *
from .robot import *
from .semantics import *
from .ros_fileserver_loader import *

from .configuration import __all__ as a
from .path_plan import __all__ as b
from .units import __all__ as c
from .robot import __all__ as d
from .semantics import __all__ as e
from .ros_fileserver_loader import __all__ as f

__all__ = a + b + c + d + e + f

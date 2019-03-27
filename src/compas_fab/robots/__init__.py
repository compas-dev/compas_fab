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
    Constraints
    PathPlan
    Robot
    RobotSemantics
    RosFileServerLoader

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
from .constraints import *
from .path_plan import *
from .units import *
from .robot import *
from .semantics import *
from .ros_fileserver_loader import *

__all__ = [name for name in dir() if not name.startswith('_')]

"""
********************************************************************************
compas_fab.robots
********************************************************************************

.. currentmodule:: compas_fab.robots

This package contains classes for robot modeling and they are used by the
simulation, planning and execution backends to exchange information.

Classses
--------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Pose
    Configuration
    Robot
    UrdfImporter
    RobotSemantics

Artists
-------

In **COMPAS**, the `artists` are classes that assist with the visualization of
datastructures and models, in a way that maintains the data separated from the
specific CAD interfaces, while providing a way to leverage native performance
of the CAD environment.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    BaseRobotArtist

Unit conversion
---------------

The following functions help with converting units from one system to the other.

.. autosummary::
    :toctree: generated/
    :nosignatures:

    to_degrees
    to_radians

"""

from .configuration import *
from .units import *
from .robot import *
from .semantics import *
from .artists import *
from .urdf_importer import *

from .configuration import __all__ as a
from .units import __all__ as b
from .robot import __all__ as c
from .semantics import __all__ as d
from .artists import __all__ as e
from .urdf_importer import __all__ as f

__all__ = a + b + c + d + e + f

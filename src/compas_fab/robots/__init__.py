"""
********************************************************************************
compas_fab.robots
********************************************************************************

.. currentmodule:: compas_fab.robots

:mod:`compas_fab.robots` contains base classes for robot simulation and control.

Classses
--------

.. autosummary::
    :toctree: generated/
    :nosignatures:

    Pose
    Configuration
    Robot
    Tool
    UrdfImporter

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

"""

from .configuration import Configuration
from .robot import Robot
from .tool import Tool
from .urdf_importer import UrdfImporter
from .artists import BaseRobotArtist

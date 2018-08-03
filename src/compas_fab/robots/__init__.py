"""
.. _compas_fab.robots:

********************************************************************************
compas_fab.robots
********************************************************************************

.. module:: compas_fab.robots

Package containing base clases for robot simulation and control.

.. autosummary::
    :toctree: generated/

    Pose
    BaseConfiguration

"""

from .configuration import BaseConfiguration
from .pose import Pose
from .robot import Robot
from .tool import Tool

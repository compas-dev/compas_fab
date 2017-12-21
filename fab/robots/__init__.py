"""
.. _compas_fab.fab.robots:

********************************************************************************
compas_fab.fab.robots
********************************************************************************

.. module:: compas_fab.fab.robots

Package containing base clases for robot simulation and control.

.. autosummary::
    :toctree: generated/

    Pose
    Robot
    BaseConfiguration
    PathPlan
    Simulator
    SimulationCoordinator
    SimulationError

"""

from .robot import BaseConfiguration, Pose
from .robot import Robot
from .tool import Tool
from .simulator import PathPlan, Simulator, SimulationCoordinator, SimulationError

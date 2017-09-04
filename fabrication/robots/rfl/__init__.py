"""
.. _compas_fabrication.fabrication.robots.rfl:

********************************************************************************
compas_fabrication.fabrication.robots.rfl
********************************************************************************

.. module:: compas_fabrication.fabrication.robots.rfl

Package with functionality to run simulations on the RFL (Robotic Fabrication
Lab) using robotic simulation tools like
`v-rep <http://www.coppeliarobotics.com/>`_.

.. autosummary::
    :toctree: generated/

    Robot
    Configuration
    Simulator
    SimulationCoordinator

"""

from .robots import Robot, Configuration, PathPlan
from .simulator import Simulator, SimulationCoordinator, SimulationError

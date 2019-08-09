"""
*******************************************************************************
compas_fab.backends.ros.plugins_choreo
*******************************************************************************

.. module:: compas_fab.backends.ros.plugins_choreo

Package with functionality to interact with `Choreo <https://github.com/yijiangh/pychoreo>`_.

Note that the transition planning module of choreo is supported by Moveit! on
`ROS <http://ros.org/>`_ through compas_fab's ROS backend.

.. autosummary::
    :toctree: generated/

    ChoreoPlanner

"""

from __future__ import absolute_import

from .ik_utils import *
from .planner_interface import *

__all__ = [name for name in dir() if not name.startswith('_')]

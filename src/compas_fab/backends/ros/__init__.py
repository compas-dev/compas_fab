"""
********************************************************************************
compas_fab.backends.ros
********************************************************************************

.. module:: compas_fab.backends.ros

Package with functionality to interact with the `ROS platform <http://ros.org/>`_.

.. autosummary::
    :toctree: generated/

    RosClient
    RosError

"""

from __future__ import absolute_import

from .messages import *
from .direct_ur_action_client import DirectUrActionClient

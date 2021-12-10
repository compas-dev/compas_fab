"""
*******************************************************************************
compas_fab.backends.kinematics
*******************************************************************************

.. module:: compas_fab.backends.kinematics

.. autosummary::
    :toctree: generated/

    AnalyticalInverseKinematics
    AnalyticalPlanCartesianMotion
    InverseKinematicsError
    CartesianMotionError

"""

from __future__ import absolute_import
from .analytical_inverse_kinematics import AnalyticalInverseKinematics  # noqa: F401
from .analytical_plan_cartesian_motion import AnalyticalPlanCartesianMotion  # noqa: F401
from .exceptions import *                 # noqa: F401,F403

__all__ = [name for name in dir() if not name.startswith('_')]

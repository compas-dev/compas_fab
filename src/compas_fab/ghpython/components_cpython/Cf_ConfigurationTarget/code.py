# r: compas_fab>=1.1.0
"""
Create a ConfigurationTarget: a target defined by joint values, not a pose.

Useful for "go to home" or "move to a specific joint configuration" motions.
Per-joint tolerances default to 0.001 m (prismatic) and 1 degree (revolute)
if not specified.

COMPAS FAB v1.1.0
"""

import math

import Grasshopper
import Rhino
import System

from compas_fab.robots import ConfigurationTarget


class ConfigurationTargetComponent(Grasshopper.Kernel.GH_ScriptInstance):
    DEFAULT_TOLERANCE_METERS = 0.001
    DEFAULT_TOLERANCE_RADIANS = math.radians(1)

    def RunScript(self, target_configuration, tolerance_above, tolerance_below):
        if target_configuration is None:
            return None

        default_above, default_below = ConfigurationTarget.generate_default_tolerances(
            target_configuration,
            self.DEFAULT_TOLERANCE_METERS,
            self.DEFAULT_TOLERANCE_RADIANS,
        )

        return ConfigurationTarget(
            target_configuration=target_configuration,
            tolerance_above=tolerance_above or default_above,
            tolerance_below=tolerance_below or default_below,
        )

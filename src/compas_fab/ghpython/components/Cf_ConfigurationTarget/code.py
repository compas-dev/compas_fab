"""
Create configuration target for the robot's end-effector motion planning.

COMPAS FAB v1.0.2
"""

import math

from ghpythonlib.componentbase import executingcomponent as component

from compas_fab.robots import ConfigurationTarget


class ConfigurationTargetComponent(component):
    DEFAULT_TOLERANCE_METERS = 0.001
    DEFAULT_TOLERANCE_RADIANS = math.radians(1)

    def RunScript(self, robot, target_configuration, tolerances_above, tolerances_below):
        if robot and target_configuration:
            default_tolerances_above, default_tolerances_below = ConfigurationTarget.generate_default_tolerances(
                target_configuration, self.DEFAULT_TOLERANCE_METERS, self.DEFAULT_TOLERANCE_RADIANS
            )

            target = ConfigurationTarget(
                target_configuration=target_configuration,
                tolerances_above=tolerances_above or default_tolerances_above,
                tolerances_below=tolerances_below or default_tolerances_below,
            )

            return target

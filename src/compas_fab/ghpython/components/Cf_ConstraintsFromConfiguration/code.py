"""
Create a position and an orientation constraint from a robot configuration.

COMPAS FAB v0.25.0
"""
import math

from ghpythonlib.componentbase import executingcomponent as component


class ConstraintsFromTargetConfiguration(component):
    def RunScript(self, robot, target_configuration, tolerance_above, tolerance_below, group_name):
        if robot and target_configuration:
            tolerance_above = tolerance_above or [math.radians(1)] * 6
            tolerance_below = tolerance_below or [math.radians(1)] * 6

            constraints = robot.constraints_from_configuration(
                configuration=target_configuration,
                tolerances_above=tolerance_above,
                tolerances_below=tolerance_below,
                group=group_name
            )

            return constraints

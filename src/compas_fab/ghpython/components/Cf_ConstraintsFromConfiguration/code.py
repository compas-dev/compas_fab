"""
Create joint constraints for each of the robot's configurable joints based on a given target configuration.

COMPAS FAB v0.26.0
"""
import math

from ghpythonlib.componentbase import executingcomponent as component


class ConstraintsFromTargetConfiguration(component):

    DEFAULT_TOLERANCE_METERS = 0.001
    DEFAULT_TOLERANCE_RADIANS = math.radians(1)

    def RunScript(self, robot, target_configuration, tolerance_above, tolerance_below, group):
        if robot and target_configuration:
            tolerance_above = tolerance_above or self._generate_default_tolerances(robot.get_configurable_joints(group))
            tolerance_below = tolerance_below or self._generate_default_tolerances(robot.get_configurable_joints(group))

            constraints = robot.constraints_from_configuration(
                configuration=target_configuration,
                tolerances_above=tolerance_above,
                tolerances_below=tolerance_below,
                group=group,
            )

            return constraints

    def _generate_default_tolerances(self, joints):
        return [self.DEFAULT_TOLERANCE_METERS if j.is_scalable() else self.DEFAULT_TOLERANCE_RADIANS for j in joints]

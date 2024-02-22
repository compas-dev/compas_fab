"""
Get a zero configuration for a robot.

COMPAS FAB v1.0.2
"""

from ghpythonlib.componentbase import executingcomponent as component


class ConfigZero(component):
    def RunScript(self, robot, group):
        if robot:
            return robot.zero_configuration(group)

        return None

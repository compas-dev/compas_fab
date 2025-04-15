# r: compas_fab>=1.0.2
"""
Get a zero configuration for a robot.

COMPAS FAB v1.0.2
"""

import Grasshopper


class ConfigZero(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot, group):
        if robot:
            return robot.zero_configuration(group)

        return None

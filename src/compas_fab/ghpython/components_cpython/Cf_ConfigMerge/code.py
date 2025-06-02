# r: compas_fab>=1.0.2
"""
Merge two configurations.

COMPAS FAB v1.1.0
"""

import Grasshopper


class ConfigMerge(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, config_a, config_b):
        if config_a and config_b:
            return config_a.merged(config_b)

        return None

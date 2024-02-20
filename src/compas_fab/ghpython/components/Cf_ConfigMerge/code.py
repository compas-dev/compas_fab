"""
Merge two configurations.

COMPAS FAB v1.0.1
"""

from ghpythonlib.componentbase import executingcomponent as component


class ConfigMerge(component):
    def RunScript(self, config_a, config_b):
        if config_a and config_b:
            return config_a.merged(config_b)

        return None

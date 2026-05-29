# r: compas_fab>=1.1.0
"""
Build a Configuration from joint values, names, and types.

Joint values must be in radians (revolute / continuous) or meters
(prismatic). Joint types follow the compas_robots.model.Joint constants:
0=fixed, 1=revolute, 2=continuous, 3=prismatic, 4=floating, 5=planar.

If `joint_types` is left empty, all joints are assumed to be revolute (1).
If `joint_names` is left empty, the configuration is unnamed (acceptable
for ConfigurationTarget but most planners expect names).

COMPAS FAB v1.1.0
"""

import Grasshopper
import System
from compas_robots import Configuration
from compas_robots.model import Joint


class ConfigurationComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        joint_values: System.Collections.Generic.List[float],
        joint_names: System.Collections.Generic.List[str],
        joint_types: System.Collections.Generic.List[int],
    ):
        if not joint_values:
            return None

        values = [float(v) for v in joint_values]
        names = [str(n) for n in joint_names] if joint_names else None
        types = [int(t) for t in joint_types] if joint_types else [Joint.REVOLUTE] * len(values)

        if names is not None and len(names) != len(values):
            raise ValueError(
                "joint_names length ({}) does not match joint_values ({})".format(len(names), len(values))
            )
        if len(types) != len(values):
            raise ValueError(
                "joint_types length ({}) does not match joint_values ({})".format(len(types), len(values))
            )

        return Configuration(joint_values=values, joint_types=types, joint_names=names)

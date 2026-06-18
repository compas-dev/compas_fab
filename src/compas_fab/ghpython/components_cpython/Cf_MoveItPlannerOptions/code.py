# r: compas_fab>=1.1.0
"""
Bundle advanced MoveIt load options into a single object for the MoveIt Planner.

All inputs are optional. Wire the `options` output into the MoveIt Planner's
`options` input. In the common case you do not need this component at all - the
MoveIt Planner falls back to sensible defaults (`/robot_description` and
`/robot_description_semantic`); this is only for non-standard ROS setups.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System

from compas_fab.ghpython import MoveItPlannerOptions


class MoveItPlannerOptionsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, urdf_param_name: str, srdf_param_name: str, http_file_server_base_url: str):
        return MoveItPlannerOptions(
            urdf_param_name=urdf_param_name or None,
            srdf_param_name=srdf_param_name or None,
            http_file_server_base_url=http_file_server_base_url or None,
        )

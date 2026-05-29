# r: compas_fab>=1.1.0
"""
Compute inverse kinematics for a target using a stateless planner.

The starting RobotCellState provides the robot's seed configuration and any
attached tools/workpieces (which affect what 'TOOL'/'WORKPIECE' target modes
resolve to). The resulting Configuration is returned; the input state is not
mutated.

If no IK solution is found, the component is flagged with an error message
(visible in the component balloon) and returns None.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import error

from compas_fab.backends.exceptions import InverseKinematicsError


class InverseKinematicsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, target, start_state, group: str):
        if not (planner and target and start_state):
            return None

        try:
            return planner.inverse_kinematics(
                target=target,
                robot_cell_state=start_state,
                group=group or None,
            )
        except InverseKinematicsError as e:
            error(ghenv.Component, "Inverse kinematics failed: {}".format(e))  # noqa: F821
            return None

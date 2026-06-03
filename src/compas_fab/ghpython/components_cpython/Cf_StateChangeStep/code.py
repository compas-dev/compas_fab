# r: compas_fab>=1.1.0
"""
Wrap an explicit RobotCellState into a named state-change PlanStep.

Use between trajectory steps in a Cf_MotionPlan to record discrete state
transitions: gripper closes, tool attaches, rigid body grasped/released.
The wired `post_state` is the state of the cell after the change has
been applied.

COMPAS FAB v1.1.0
"""

import Grasshopper

from compas_fab.robots import PlanStep


class StateChangeStep(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, post_state, description: str):
        if not name or post_state is None:
            return None
        return PlanStep(name=name.strip(), post_state=post_state, description=(description or "").strip())

# r: compas_fab>=1.1.0
"""
Wrap an explicit RobotCellState into a named state-change Action.

Use between trajectory actions in a Cf_ActionChain to record discrete
state transitions: gripper closes, tool attaches, rigid body
grasped/released. The wired `post_state` is the state of the cell after
the change has been applied. Optional `tags` (CSV) drive differentiated
downstream execution.

COMPAS FAB v1.1.0
"""


import Grasshopper
import Rhino
import System

from compas_fab.robots import Action


class StateChangeAction(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, post_state, tags: str, description: str):
        if not name or post_state is None:
            return None
        tag_list = [t.strip() for t in (tags or "").split(",") if t.strip()]
        return Action(
            name=name.strip(),
            post_state=post_state,
            tags=tag_list,
            description=(description or "").strip(),
        )

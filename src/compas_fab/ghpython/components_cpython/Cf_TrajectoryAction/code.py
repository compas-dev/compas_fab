# r: compas_fab>=2.0.0
"""
Wrap a JointTrajectory into a named Action, ready for Cf_ActionChain.

Use this between a planner output (Plan Motion / Plan Cartesian Motion)
and the Cf_ActionChain assembler: drop one per trajectory in the chain,
type the name next to the trajectory it labels. Optional `tags` (CSV)
drive differentiated downstream execution (e.g. approach, linear).

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System

from compas_fab.robots import Action


class TrajectoryAction(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, trajectory, tags: str, description: str):
        if not name or trajectory is None:
            return None
        tag_list = [t.strip() for t in (tags or "").split(",") if t.strip()]
        return Action(
            name=name.strip(),
            trajectory=trajectory,
            tags=tag_list,
            description=(description or "").strip(),
        )

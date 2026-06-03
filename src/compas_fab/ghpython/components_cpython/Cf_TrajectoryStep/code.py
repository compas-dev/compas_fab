# r: compas_fab>=1.1.0
"""
Wrap a JointTrajectory into a named PlanStep, ready for Cf_MotionPlan.

Use this between a planner output (Plan Motion / Plan Cartesian Motion)
and the Cf_MotionPlan assembler — drop one per trajectory in the plan,
type the name next to the trajectory it labels.

COMPAS FAB v1.1.0
"""

import Grasshopper

from compas_fab.robots import PlanStep


class TrajectoryStep(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, trajectory, description: str):
        if not name or trajectory is None:
            return None
        return PlanStep(name=name.strip(), trajectory=trajectory, description=(description or "").strip())

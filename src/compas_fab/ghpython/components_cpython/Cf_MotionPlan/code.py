# r: compas_fab>=1.1.0
"""
Assemble a MotionPlan from a list of PlanSteps and emit the composite trajectory visualisation.

Wire the outputs of Cf_TrajectoryStep / Cf_StateChangeStep components into
`steps` in the order the motion should be executed. The component threads
the cell state through the chain, validates each step, and emits the
assembled plan plus the composite EE planes / polyline for visualisation.

`planes` is a DataTree with one branch per trajectory step (so per-step
coloring and filtering work out of the box); `polyline` is a list of
polylines parallel to those branches. Flatten the tree for a single
combined list.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import error as gh_error
from ghpythonlib.treehelpers import list_to_tree

from compas_fab.ghpython import trajectory_to_planes_and_polyline
from compas_fab.robots import MotionPlan


class MotionPlanComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, start_state, steps, robot_cell, group: str, description: str):
        if not name or start_state is None or not steps:
            return (None, [], None, 0.0, start_state)

        try:
            plan = MotionPlan(
                name=name.strip(),
                start_state=start_state,
                robot_cell=robot_cell,
                description=(description or "").strip(),
            )
            for step in steps:
                if step is None:
                    continue
                plan.append_step(step)
        except ValueError as e:
            gh_error(ghenv.Component, "Plan assembly failed: {}".format(e))  # noqa: F821
            return (None, [], None, 0.0, start_state)

        if robot_cell is None:
            return (plan, [], None, plan.duration, plan.end_state)

        # Composite visualisation: one branch per trajectory step. Walk the
        # chain so each step's FK uses the correct pre-state.
        planes_per_step = []
        polylines_per_step = []
        running_state = plan.start_state
        for step in plan:
            if not step.is_trajectory:
                running_state = step.post_state
                continue
            planes, polyline = trajectory_to_planes_and_polyline(
                robot_cell, running_state, step.trajectory, group or None
            )
            planes_per_step.append(planes)
            polylines_per_step.append(polyline)
            running_state = step.post_state

        planes_tree = list_to_tree(planes_per_step) if planes_per_step else []
        return (plan, planes_tree, polylines_per_step or None, plan.duration, plan.end_state)

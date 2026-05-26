# r: compas_fab>=1.1.0
"""
Plan a free-space (joint-interpolated) motion to a target.

Calls `planner.plan_motion(target, start_state)`. The result is cached in
sticky so the canvas does not re-plan on every refresh - set `compute` to
True to (re)plan.

Catches the common motion-planning exceptions and surfaces a `partial_trajectory`
when the planner could provide one.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.backends import MotionPlanningError


class PlanMotion(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        planner,
        target,
        start_state,
        group: str,
        planner_id: str,
        num_planning_attempts: int,
        allowed_planning_time: float,
        compute: bool,
    ):
        key = create_id(ghenv.Component, "trajectory")  # noqa: F821

        if not (planner and target and start_state and compute):
            cached = st.get(key)
            if cached is None:
                return (None, None)
            return cached

        options = {}
        if planner_id:
            options["planner_id"] = planner_id
        if num_planning_attempts:
            options["num_planning_attempts"] = int(num_planning_attempts)
        if allowed_planning_time:
            options["allowed_planning_time"] = float(allowed_planning_time)

        try:
            trajectory = planner.plan_motion(
                target=target,
                start_state=start_state,
                group=group or None,
                options=options or None,
            )
            error = None
        except MotionPlanningError as e:
            print("Motion planning failed: {}".format(e))
            trajectory = getattr(e, "partial_trajectory", None)
            error = str(e)

        st[key] = (trajectory, error)
        return st[key]

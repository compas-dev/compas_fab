# r: compas_fab>=1.1.0
"""
Plan a free-space (joint-interpolated) motion to a target.

Calls `planner.plan_motion(target, start_state)`. The result is cached in
sticky so the canvas does not re-plan on every refresh - set `compute` to
True to (re)plan.

On `MotionPlanningError` the partial trajectory (if any) is surfaced and the
component is flagged with a warning. If no partial trajectory is available,
the component is flagged with an error.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from compas_ghpython import error as gh_error
from compas_ghpython import warning as gh_warning
from scriptcontext import sticky as st

from compas_fab.backends import MotionPlanningError
from compas_fab.ghpython import trajectory_to_planes_and_polyline


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

        def _viz(trajectory):
            return trajectory_to_planes_and_polyline(planner, start_state, trajectory, group or None)

        if not (planner and target and start_state and compute):
            cached = st.get(key)
            if cached is None:
                return (None, None, [], None)
            trajectory, error_msg = cached
            planes, polyline = _viz(trajectory)
            return (trajectory, error_msg, planes, polyline)

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
            error_msg = None
        except MotionPlanningError as e:
            trajectory = getattr(e, "partial_trajectory", None)
            error_msg = str(e)
            if trajectory is not None:
                gh_warning(ghenv.Component, "Motion planning failed but a partial trajectory was returned: {}".format(error_msg))  # noqa: F821
            else:
                gh_error(ghenv.Component, "Motion planning failed: {}".format(error_msg))  # noqa: F821

        st[key] = (trajectory, error_msg)
        planes, polyline = _viz(trajectory)
        return (trajectory, error_msg, planes, polyline)

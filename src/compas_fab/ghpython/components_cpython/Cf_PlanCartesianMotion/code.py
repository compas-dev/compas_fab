# r: compas_fab>=1.1.0
"""
Plan a Cartesian (linear-in-tool-space) motion through waypoints.

Calls `planner.plan_cartesian_motion(waypoints, start_state)`. The result is
cached in sticky; toggle `compute` to (re)plan.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.backends import MotionPlanningError


class PlanCartesianMotion(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        planner,
        waypoints,
        start_state,
        group: str,
        max_step: float,
        avoid_collisions: bool,
        compute: bool,
    ):
        key = create_id(ghenv.Component, "trajectory")  # noqa: F821

        if not (planner and waypoints and start_state and compute):
            cached = st.get(key)
            if cached is None:
                return (None, None, None)
            return cached

        options = {}
        if max_step:
            options["max_step"] = float(max_step)
        if avoid_collisions is not None:
            options["avoid_collisions"] = bool(avoid_collisions)

        try:
            trajectory = planner.plan_cartesian_motion(
                waypoints=waypoints,
                start_state=start_state,
                group=group or None,
                options=options or None,
            )
            error = None
        except MotionPlanningError as e:
            print("Cartesian motion planning failed: {}".format(e))
            trajectory = getattr(e, "partial_trajectory", None)
            error = str(e)

        fraction = getattr(trajectory, "fraction", None) if trajectory is not None else None
        st[key] = (trajectory, fraction, error)
        return st[key]

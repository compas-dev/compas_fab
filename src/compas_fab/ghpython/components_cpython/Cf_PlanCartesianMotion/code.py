# r: compas_fab>=1.1.0
"""
Plan a Cartesian (linear-in-tool-space) motion through waypoints.

Calls `planner.plan_cartesian_motion(waypoints, start_state)`. The result is
cached in sticky; toggle `compute` to (re)plan.

Any planning failure (no plan found, start/goal in collision, …) flags the
component red with the backend error message. When the backend returns a
`partial_trajectory`, the component is flagged with a warning instead and
the partial trajectory is surfaced.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import create_id
from compas_ghpython import error as gh_error
from compas_ghpython import warning as gh_warning
from scriptcontext import sticky as st

from compas_fab.backends import BackendError
from compas_fab.backends import MotionPlanningError
from compas_fab.ghpython import trajectory_to_planes_and_polyline


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

        def _viz(trajectory):
            return trajectory_to_planes_and_polyline(planner, start_state, trajectory, group or None)

        def _replay_marker(error_msg, has_trajectory):
            if not error_msg:
                return
            if has_trajectory:
                gh_warning(ghenv.Component, "Cartesian motion planning failed but a partial trajectory was returned: {}".format(error_msg))  # noqa: F821
            else:
                gh_error(ghenv.Component, "Cartesian motion planning failed: {}".format(error_msg))  # noqa: F821

        if not (planner and waypoints and start_state and compute):
            cached = st.get(key)
            if cached is None:
                return (None, None, [], None)
            trajectory, fraction, error_msg = cached
            _replay_marker(error_msg, trajectory is not None)
            planes, polyline = _viz(trajectory)
            return (trajectory, fraction, planes, polyline)

        options = {}
        if max_step:
            options["max_step"] = float(max_step)
        if avoid_collisions is not None:
            options["avoid_collisions"] = bool(avoid_collisions)

        trajectory = None
        error_msg = None
        try:
            trajectory = planner.plan_cartesian_motion(
                waypoints=waypoints,
                start_state=start_state,
                group=group or None,
                options=options or None,
            )
        except MotionPlanningError as e:
            trajectory = getattr(e, "partial_trajectory", None)
            error_msg = str(e)
        except BackendError as e:
            error_msg = str(e)
        _replay_marker(error_msg, trajectory is not None)

        fraction = getattr(trajectory, "fraction", None) if trajectory is not None else None
        st[key] = (trajectory, fraction, error_msg)
        planes, polyline = _viz(trajectory)
        return (trajectory, fraction, planes, polyline)

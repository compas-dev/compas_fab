# r: compas_fab>=1.1.0
"""
Plan a free-space (joint-interpolated) motion to a target.

Calls `planner.plan_motion(target, start_state)`. The result is cached in
sticky so the canvas does not re-plan on every refresh - set `compute` to
True to (re)plan.

Any planning failure (no plan found, start/goal in collision, timed out, …)
flags the component red with the backend error message. When the backend
returns a `partial_trajectory`, the component is flagged with a warning
instead and the partial trajectory is surfaced.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas_ghpython import create_id
from compas_ghpython import error as gh_error
from compas_ghpython import warning as gh_warning
from compas_rhino.conversions import frame_to_rhino_plane
from compas_rhino.conversions import polyline_to_rhino
from scriptcontext import sticky as st

from compas_fab.backends import BackendError
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

        def _viz(trajectory):
            frames, polyline = trajectory.to_frames_and_polyline(planner.robot_cell, start_state, group or None)
            planes = [frame_to_rhino_plane(f) for f in frames]
            polyline = polyline_to_rhino(polyline) if polyline else None
            return planes, polyline

        def _replay_marker(error_msg, has_trajectory):
            if not error_msg:
                return
            if has_trajectory:
                gh_warning(ghenv.Component, "Motion planning failed but a partial trajectory was returned: {}".format(error_msg))  # noqa: F821
            else:
                gh_error(ghenv.Component, "Motion planning failed: {}".format(error_msg))  # noqa: F821

        if not (planner and target and start_state and compute):
            cached = st.get(key)
            if cached is None:
                return (None, [], None)
            trajectory, error_msg = cached
            _replay_marker(error_msg, trajectory is not None)
            planes, polyline = _viz(trajectory)
            return (trajectory, planes, polyline)

        options = {}
        if planner_id:
            options["planner_id"] = planner_id
        if num_planning_attempts:
            options["num_planning_attempts"] = int(num_planning_attempts)
        if allowed_planning_time:
            options["allowed_planning_time"] = float(allowed_planning_time)

        trajectory = None
        error_msg = None
        try:
            trajectory = planner.plan_motion(
                target=target,
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

        st[key] = (trajectory, error_msg)
        planes, polyline = _viz(trajectory)
        return (trajectory, planes, polyline)

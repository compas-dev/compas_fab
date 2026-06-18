# r: compas_fab>=1.1.0
"""
Plan a free-space (joint-interpolated) motion to a target.

Calls `planner.plan_motion(target, start_state)`. The result is cached in
sticky so the canvas does not re-plan on every refresh - set `compute` to
True to (re)plan.

Any planning failure (no plan found, start/goal in collision, timed out, …)
flags the component red with the backend error message. When the backend
returns a `partial_trajectory`, the component is flagged with a warning
instead and the partial trajectory is surfaced. On a full failure the start
state is collision-checked (when the planner supports it) and a concise hint
appended. The message is also exposed on the `debug_info` output so it can be
read in a wired panel.

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
from compas_fab.ghpython import collision_diagnostic


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

        def _viz(trajectory, target_mode):
            if trajectory is None:
                return [], None
            frames, polyline = trajectory.to_frames_and_polyline(planner.robot_cell, start_state, group or None, target_mode)
            planes = [frame_to_rhino_plane(f) for f in frames]
            polyline = polyline_to_rhino(polyline) if polyline else None
            return planes, polyline

        def _diagnose(error_msg, has_trajectory):
            """Flag the component for an error and return the message shown (for debug_info)."""
            if not error_msg:
                return ""
            if has_trajectory:
                message = "Motion planning failed but a partial trajectory was returned: {}".format(error_msg)
                gh_warning(ghenv.Component, message)  # noqa: F821
            else:
                message = "Motion planning failed: {}".format(error_msg)
                gh_error(ghenv.Component, message)  # noqa: F821
            return message

        if not (planner and target and start_state and compute):
            cached = st.get(key)
            if cached is None:
                return (None, [], None, "")
            trajectory, error_msg, target_mode = cached
            debug_info = _diagnose(error_msg, trajectory is not None)
            planes, polyline = _viz(trajectory, target_mode)
            return (trajectory, planes, polyline, debug_info)

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

        # On a full failure, append a concise collision check on the start state
        # (a common cause). Cached with the error so replays don't re-query the backend.
        if error_msg and trajectory is None:
            hint = collision_diagnostic(planner, start_state, group or None)
            if hint:
                error_msg += "\n" + hint

        target_mode = getattr(target, "target_mode", None)
        debug_info = _diagnose(error_msg, trajectory is not None)
        st[key] = (trajectory, error_msg, target_mode)
        planes, polyline = _viz(trajectory, target_mode)
        return (trajectory, planes, polyline, debug_info)

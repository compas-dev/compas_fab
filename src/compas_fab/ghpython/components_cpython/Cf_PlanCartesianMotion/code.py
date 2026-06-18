# r: compas_fab>=2.0.1
"""
Plan a Cartesian (linear-in-tool-space) motion through waypoints.

Calls `planner.plan_cartesian_motion(waypoints, start_state)`. The result is
cached in sticky; toggle `compute` to (re)plan.

Any planning failure (no plan found, start/goal in collision, …) flags the
component red with the backend error message. When the backend returns a
`partial_trajectory`, the component is flagged with a warning instead and
the partial trajectory is surfaced. On a full failure the start state is
collision-checked (when the planner supports it) and a concise hint appended.
The message is also exposed on the `debug_info` output for reading in a panel.

COMPAS FAB v2.0.1
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
from compas_fab.robots import JointTrajectory


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

        def _viz(trajectory: JointTrajectory, target_mode):
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
                message = "Cartesian motion planning failed but a partial trajectory was returned: {}".format(error_msg)
                gh_warning(ghenv.Component, message)  # noqa: F821
            else:
                message = "Cartesian motion planning failed: {}".format(error_msg)
                gh_error(ghenv.Component, message)  # noqa: F821
            return message

        if not (planner and waypoints and start_state and compute):
            cached = st.get(key)
            if cached is None:
                return (None, None, [], None, "")
            trajectory, fraction, error_msg, target_mode = cached
            debug_info = _diagnose(error_msg, trajectory is not None)
            planes, polyline = _viz(trajectory, target_mode)
            return (trajectory, fraction, planes, polyline, debug_info)

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

        # On a full failure, append a concise collision check on the start state.
        # Cached with the error so replays don't re-query the backend.
        if error_msg and trajectory is None:
            hint = collision_diagnostic(planner, start_state, group or None)
            if hint:
                error_msg += "\n" + hint

        target_mode = getattr(waypoints, "target_mode", None)
        debug_info = _diagnose(error_msg, trajectory is not None)
        fraction = trajectory.fraction if trajectory is not None else None
        st[key] = (trajectory, fraction, error_msg, target_mode)
        planes, polyline = _viz(trajectory, target_mode)
        return (trajectory, fraction, planes, polyline, debug_info)

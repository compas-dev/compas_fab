# r: compas_fab>=1.1.0
"""
Compute inverse kinematics for a target using a stateless planner.

`target` accepts either a proper Target object (`FrameTarget`,
`PointAxisTarget`, `ConfigurationTarget`, ...) or a bare `compas.geometry.Frame`
or Rhino `Plane`. In the latter two cases the component auto-wraps the input
as a `FrameTarget(target_mode=ROBOT)` with default tolerances and emits a
warning. Wire a dedicated Target component for non-default settings.

The starting RobotCellState provides the robot's seed configuration and any
attached tools/workpieces (which affect what 'TOOL'/'WORKPIECE' target modes
resolve to). The resulting Configuration is returned; the input state is not
mutated.

If `start_state` is left unwired, a zero-configuration default state is
derived from the planner's robot_cell and used as the seed; a warning is
surfaced on the component so the user knows attachments aren't being
considered. Wire an explicit `start_state` to pose the robot mid-process
or to use a state with attached tools / workpieces.

If no IK solution is found, the component is flagged with an error message
(visible in the component balloon) and returns None.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas.geometry import Frame
from compas_ghpython import error
from compas_ghpython import warning
from compas_rhino.conversions import plane_to_compas_frame

from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState
from compas_fab.robots import Target
from compas_fab.robots import TargetMode


class InverseKinematicsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, target, start_state, group: str):
        if not (planner and target):
            return None

        if not isinstance(target, Target):
            if isinstance(target, Frame):
                target = FrameTarget(target_frame=target, target_mode=TargetMode.ROBOT)
                warning(ghenv.Component, "Wrapped a bare COMPAS Frame as FrameTarget(target_mode=ROBOT) with default tolerances. Wire a Target component for non-default settings.")  # noqa: F821
            else:
                # Anything else (Rhino Plane, GH_Plane wrapper, ...) — try the
                # plane-to-frame conversion. If it isn't plane-shaped, surface
                # a clean error.
                try:
                    frame = plane_to_compas_frame(target)
                except Exception:
                    error(ghenv.Component, "target must be a Target, a COMPAS Frame or a Rhino Plane; got {}.".format(type(target).__name__))  # noqa: F821
                    return None
                target = FrameTarget(target_frame=frame, target_mode=TargetMode.ROBOT)
                warning(ghenv.Component, "Wrapped a Rhino Plane as FrameTarget(target_mode=ROBOT) with default tolerances. Wire a Target component for non-default settings.")  # noqa: F821

        if start_state is None:
            if planner.robot_cell is None:
                error(ghenv.Component, "No start_state wired and the planner has no robot_cell to derive one from.")  # noqa: F821
                return None
            start_state = planner.robot_cell_state or RobotCellState.from_robot_cell(planner.robot_cell)
            warning(ghenv.Component, "No start_state wired; using zero-configuration default from planner.robot_cell. Wire a state explicitly to seed from a different configuration or to keep tool/workpiece attachments.")  # noqa: F821

        try:
            return planner.inverse_kinematics(
                target=target,
                robot_cell_state=start_state,
                group=group or None,
            )
        except InverseKinematicsError as e:
            error(ghenv.Component, "Inverse kinematics failed: {}".format(e))  # noqa: F821
            return None

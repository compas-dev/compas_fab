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
resolve to). The component returns both the resulting Configuration and a
copy of the start_state with that Configuration applied (`cell_state`), so
the next stage (Visualize / FK / next planner step) can wire directly
without an intermediate SetRobotConfiguration. The input state is not
mutated.

If `start_state` is left unwired, a zero-configuration default state is
derived from the planner's robot_cell and used as the seed; a warning is
surfaced on the component so the user knows attachments aren't being
considered. Wire an explicit `start_state` to pose the robot mid-process
or to use a state with attached tools / workpieces.

If no IK solution is found, the component is flagged with an error message
(visible in the component balloon) and returns (None, None).

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
from compas.geometry import Frame
from compas_ghpython import error
from compas_ghpython import warning
from compas_rhino.conversions import plane_to_compas_frame

from compas_fab.backends.exceptions import BackendTargetNotSupportedError
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState
from compas_fab.robots import Target
from compas_fab.robots import TargetMode


class InverseKinematicsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, target, start_state, group: str, check_collision: bool):
        if planner is None or target is None:
            # Warn only when the input *is* wired but the upstream returned
            # None (i.e. upstream silently failed). If nothing is wired, the
            # user is still building the canvas: staying quiet keeps it clean.
            for name, value in (("planner", planner), ("target", target)):
                if value is None:
                    param = next(p for p in ghenv.Component.Params.Input if p.Name == name)  # noqa: F821
                    if param.SourceCount > 0:
                        warning(ghenv.Component, "{} input is wired but received None; check the upstream component for errors.".format(name))  # noqa: F821
            return (None, None)

        if not isinstance(target, Target):
            wrap_msg = "Wrapped a {} as FrameTarget(target_mode=ROBOT) with default tolerances. Wire a Target component for non-default settings."
            if isinstance(target, Frame):
                target = FrameTarget(target_frame=target, target_mode=TargetMode.ROBOT)
                warning(ghenv.Component, wrap_msg.format("bare COMPAS Frame"))  # noqa: F821
            else:
                # Anything else (Rhino Plane, GH_Plane wrapper, ...): try the
                # plane-to-frame conversion. If it isn't plane-shaped, surface
                # a clean error.
                try:
                    frame = plane_to_compas_frame(target)
                except Exception:
                    error(ghenv.Component, "target must be a Target, a COMPAS Frame or a Rhino Plane; got {}.".format(type(target).__name__))  # noqa: F821
                    return (None, None)
                target = FrameTarget(target_frame=frame, target_mode=TargetMode.ROBOT)
                warning(ghenv.Component, wrap_msg.format("Rhino Plane"))  # noqa: F821

        if start_state is None:
            if planner.robot_cell is None:
                error(ghenv.Component, "No start_state wired and the planner has no robot_cell to derive one from.")  # noqa: F821
                return (None, None)
            start_state = planner.robot_cell_state or RobotCellState.from_robot_cell(planner.robot_cell)
            warning(  # noqa: F821
                ghenv.Component,  # noqa: F821
                "No start_state wired; using zero-configuration default from planner.robot_cell. "
                "Wire a state explicitly to seed from a different configuration or to keep tool/workpiece attachments.",
            )

        options = None
        if check_collision is not None:
            options = {"check_collision": bool(check_collision)}

        try:
            configuration = planner.inverse_kinematics(
                target=target,
                robot_cell_state=start_state,
                group=group or None,
                options=options,
            )
        except BackendTargetNotSupportedError:
            # The exception is raised with no payload: surface something
            # actionable: name the target type and the planner that rejected it.
            error(  # noqa: F821
                ghenv.Component,  # noqa: F821
                "Planner '{}' does not support target type '{}'. "
                "Use a target type supported by this backend (typically `FrameTarget` "
                "for the Analytical planner) or switch backends.".format(
                    type(planner).__name__, type(target).__name__),
            )
            return (None, None)
        except InverseKinematicsError as e:
            error(ghenv.Component, "Inverse kinematics failed: {}".format(e))  # noqa: F821
            return (None, None)

        if configuration is None:
            return (None, None)

        new_state = deepcopy(start_state)
        new_state.robot_configuration = configuration
        return (configuration, new_state)

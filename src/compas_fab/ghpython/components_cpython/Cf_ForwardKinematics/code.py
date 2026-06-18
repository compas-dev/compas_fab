# r: compas_fab>=2.0.0
"""
Compute forward kinematics from a RobotCellState using a stateless planner.

The returned frame depends on the `target_mode`:
* 'ROBOT'      : PCF (planner coordinate frame, last link of the planning group)
* 'TOOL'       : TCF of the attached tool
* 'WORKPIECE'  : OCF of the attached workpiece

The Configuration to evaluate is taken from `cell_state.robot_configuration`;
use the SetRobotConfiguration helper upstream to override it.

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System
from compas_rhino.conversions import frame_to_rhino_plane

from compas_fab.ghpython import ensure_value_list
from compas_fab.robots import TargetMode

_TARGET_MODES = [mode.value for mode in TargetMode]


class ForwardKinematicsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, planner, cell_state, target_mode: str, group: str):
        ensure_value_list(ghenv.Component, "target_mode", _TARGET_MODES, default="ROBOT")  # noqa: F821

        if not (planner and cell_state):
            return (None, None)

        mode = target_mode or "ROBOT"
        if isinstance(mode, str):
            mode = TargetMode(mode.strip().upper())

        frame = planner.forward_kinematics(
            robot_cell_state=cell_state,
            target_mode=mode,
            group=group or None,
        )
        plane = frame_to_rhino_plane(frame)
        return (frame, plane)

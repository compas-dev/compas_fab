# r: compas_fab>=2.0.0
"""
Create a FrameTarget for the robot's end-effector motion planning.

Accepts a Rhino plane or a COMPAS Frame. The `target_mode` controls which
frame on the robot/tool/workpiece is matched against the target.

COMPAS FAB v2.0.0
"""

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame

from compas_fab.ghpython import ensure_value_list
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

_TARGET_MODES = [mode.value for mode in TargetMode]


class FrameTargetComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        plane,
        target_mode: str,
        tolerance_position: float,
        tolerance_orientation: float,
        native_scale: float,
    ):
        ensure_value_list(ghenv.Component, "target_mode", _TARGET_MODES, default="ROBOT")  # noqa: F821

        if plane is None:
            return None

        frame = plane if isinstance(plane, Frame) else plane_to_compas_frame(plane)

        mode = target_mode or "ROBOT"
        if isinstance(mode, str):
            mode = TargetMode(mode.strip().upper())

        native_scale = native_scale if native_scale else 1.0

        target = FrameTarget(
            target_frame=frame,
            target_mode=mode,
            native_scale=native_scale,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )
        return target

# r: compas_fab>=1.1.0
"""
Create FrameWaypoints from a list of Rhino planes (or COMPAS Frames).

Used as input to PlanCartesianMotion to interpolate linearly through a
sequence of fully-constrained poses.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System
from compas.geometry import Frame
from compas_rhino.conversions import plane_to_compas_frame

from compas_fab.ghpython import ensure_value_list
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import TargetMode

_TARGET_MODES = [mode.value for mode in TargetMode]


class FrameWaypointsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        planes: System.Collections.Generic.List[object],
        target_mode: str,
        tolerance_position: float,
        tolerance_orientation: float,
        native_scale: float,
    ):
        ensure_value_list(ghenv.Component, "target_mode", _TARGET_MODES, default="ROBOT")  # noqa: F821

        if not planes:
            return None

        frames = []
        for plane in planes:
            if plane is None:
                continue
            frames.append(plane if isinstance(plane, Frame) else plane_to_compas_frame(plane))

        if not frames:
            return None

        mode = target_mode or "ROBOT"
        if isinstance(mode, str):
            mode = TargetMode(mode.strip().upper())

        native_scale = native_scale if native_scale else 1.0

        return FrameWaypoints(
            target_frames=frames,
            target_mode=mode,
            native_scale=native_scale,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )

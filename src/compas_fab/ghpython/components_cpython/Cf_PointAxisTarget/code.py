# r: compas_fab>=1.1.0
"""
Create a PointAxisTarget: a position with a free rotation around the Z axis.

Useful for cylindrically symmetric tools (drilling, milling, 3D printing) where
rotation around the tool axis is unconstrained.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import rhinoscriptsyntax as rs
import System
from compas.geometry import Point
from compas.geometry import Vector
from compas_rhino.conversions import point_to_compas
from compas_rhino.conversions import vector_to_compas

from compas_fab.ghpython import ensure_value_list
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import TargetMode

_TARGET_MODES = [mode.value for mode in TargetMode]


class PointAxisTargetComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        point,
        target_z_axis,
        target_mode: str,
        tolerance_position: float,
        tolerance_orientation: float,
        native_scale: float,
    ):
        ensure_value_list(ghenv.Component, "target_mode", _TARGET_MODES, default="ROBOT")  # noqa: F821

        if point is None or target_z_axis is None:
            return None

        cpoint = point if isinstance(point, Point) else point_to_compas(rs.coerce3dpoint(point))
        caxis = target_z_axis if isinstance(target_z_axis, Vector) else vector_to_compas(rs.coerce3dvector(target_z_axis))

        mode = target_mode or "ROBOT"
        if isinstance(mode, str):
            mode = TargetMode(mode.strip().upper())

        native_scale = native_scale if native_scale else 1.0

        return PointAxisTarget(
            target_point=cpoint,
            target_z_axis=caxis,
            target_mode=mode,
            native_scale=native_scale,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )

# r: compas_fab>=1.1.0
"""
Create PointAxisWaypoints from parallel lists of points and Z-axes.

Useful for cylindrically symmetric tools (drilling, milling, 3D printing)
following a sequence of targets where rotation around the tool axis is
unconstrained. The `points` and `axes` lists must have the same length.

COMPAS FAB v1.1.0
"""

import Grasshopper
import Rhino
import System  # noqa: F401
from compas.geometry import Point
from compas.geometry import Vector
from compas_ghpython import error
from compas_rhino.conversions import point_to_compas
from compas_rhino.conversions import vector_to_compas

from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import TargetMode


class PointAxisWaypointsComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        points: list[Rhino.Geometry.Point3d],
        axes: list[object],
        target_mode: str,
        tolerance_position: float,
        tolerance_orientation: float,
        native_scale: float,
    ):
        if not points or not axes:
            return None
        if len(points) != len(axes):
            error(ghenv.Component, "Number of points ({}) and axes ({}) must match".format(len(points), len(axes)))  # noqa: F821
            return None

        pairs = []
        for p, a in zip(points, axes):
            if p is None or a is None:
                continue
            cp = p if isinstance(p, Point) else point_to_compas(p)
            ca = a if isinstance(a, Vector) else vector_to_compas(a)
            pairs.append((cp, ca.unitized()))

        if not pairs:
            return None

        mode = target_mode or "ROBOT"
        if isinstance(mode, str):
            mode = TargetMode(mode.strip().upper())

        native_scale = native_scale if native_scale else 1.0

        return PointAxisWaypoints(
            target_points_and_axes=pairs,
            target_mode=mode,
            native_scale=native_scale,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )

"""
Create a point and axis target for the robot's end-effector motion planning.

COMPAS FAB v1.0.2
"""

from compas_rhino.conversions import point_to_compas
from compas_rhino.conversions import vector_to_compas
from ghpythonlib.componentbase import executingcomponent as component

from compas.geometry import Point
from compas.geometry import Vector
from compas_fab.robots import PointAxisTarget


class PointAxisTargetComponent(component):
    def RunScript(
        self, point, target_z_axis, target_mode, tolerance_position, tolerance_orientation, tool_coordinate_frame
    ):
        target = None
        if point:

            # Convert Rhino geometry to COMPAS geometry
            point = point if isinstance(point, Point) else point_to_compas(point)
            target_z_axis = target_z_axis if isinstance(target_z_axis, Vector) else vector_to_compas(target_z_axis)

            target = PointAxisTarget(
                point, target_z_axis, target_mode, tolerance_position, tolerance_orientation, tool_coordinate_frame
            )

        return target

"""
Create a position and an orientation constraint from a plane calculated for the group's end-effector link.

COMPAS FAB v1.0.2
"""

import math

from compas_rhino.conversions import point_to_compas
from compas_rhino.conversions import vector_to_compas
from ghpythonlib.componentbase import executingcomponent as component

from compas.geometry import Point
from compas.geometry import Vector
from compas_fab.robots import PointAxisTarget


class PointAxisTargetComponent(component):
    def RunScript(self, point, target_z_vector, tolerance_position, tool_coordinate_frame):
        target = None
        if point:

            # Convert Rhino geometry to COMPAS geometry
            point = point if isinstance(point, Point) else point_to_compas(point)
            target_z_vector = (
                target_z_vector if isinstance(target_z_vector, Vector) else vector_to_compas(target_z_vector)
            )

            target = PointAxisTarget(point, target_z_vector, tolerance_position, tool_coordinate_frame)

        return target

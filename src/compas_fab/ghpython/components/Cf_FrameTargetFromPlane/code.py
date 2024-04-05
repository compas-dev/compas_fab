"""
Create a position and an orientation constraint from a plane calculated for the group's end-effector link.

COMPAS FAB v1.0.2
"""

import math

from compas_rhino.conversions import plane_to_compas_frame
from ghpythonlib.componentbase import executingcomponent as component

from compas.geometry import Frame
from compas_fab.robots import FrameTarget


class FrameTargetFromPlaneComponent(component):
    def RunScript(
        self, plane, tolerance_position, tolerance_xaxis, tolerance_yaxis, tolerance_zaxis, tool_coordinate_frame
    ):
        target = None
        if plane:
            # Convert Rhino geometry to COMPAS geometry
            frame = plane
            if not isinstance(frame, Frame):
                frame = plane_to_compas_frame(frame)
            if not isinstance(tool_coordinate_frame, Frame):
                tool_coordinate_frame = plane_to_compas_frame(tool_coordinate_frame)

            # Tolerance values
            tolerance_position = tolerance_position or 0.001
            tolerance_xaxis = tolerance_xaxis or 1.0
            tolerance_yaxis = tolerance_yaxis or 1.0
            tolerance_zaxis = tolerance_zaxis or 1.0
            tolerance_orientation = [
                math.radians(tolerance_xaxis),
                math.radians(tolerance_yaxis),
                math.radians(tolerance_zaxis),
            ]

            target = FrameTarget(frame, tolerance_position, tolerance_orientation, tool_coordinate_frame)

        return target

"""
Create a fully constrained pose target for the robot's end-effector using a GH Plane or compas Frame.

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
            DEFAULT_TOLERANCE_METERS = 0.001
            DEFAULT_TOLERANCE_RADIANS = math.radians(1)
            tolerance_position = tolerance_position or DEFAULT_TOLERANCE_METERS
            tolerance_xaxis = tolerance_xaxis or DEFAULT_TOLERANCE_RADIANS
            tolerance_yaxis = tolerance_yaxis or DEFAULT_TOLERANCE_RADIANS
            tolerance_zaxis = tolerance_zaxis or DEFAULT_TOLERANCE_RADIANS
            tolerance_orientation = [
                (tolerance_xaxis),
                (tolerance_yaxis),
                (tolerance_zaxis),
            ]

            target = FrameTarget(frame, tolerance_position, tolerance_orientation, tool_coordinate_frame)

        return target

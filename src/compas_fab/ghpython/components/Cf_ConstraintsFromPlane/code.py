"""
Create a position and an orientation constraint from a plane calculated for the group's end-effector link.

COMPAS FAB v0.21.0
"""
import math

from ghpythonlib.componentbase import executingcomponent as component

from compas_fab.ghpython.components import coerce_frame


class ConstraintsFromPlane(component):
    def RunScript(self, robot, plane, group, tolerance_position, tolerance_xaxis, tolerance_yaxis, tolerance_zaxis):
        goal_constraints = None
        if robot and plane:
            tolerance_position = tolerance_position or 0.001
            tolerance_xaxis = tolerance_xaxis or 1.
            tolerance_yaxis = tolerance_yaxis or 1.
            tolerance_zaxis = tolerance_zaxis or 1.

            frame = coerce_frame(plane)
            tolerances_axes = [math.radians(tolerance_xaxis), math.radians(tolerance_yaxis), math.radians(tolerance_zaxis)]
            goal_constraints = robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group)

        return goal_constraints

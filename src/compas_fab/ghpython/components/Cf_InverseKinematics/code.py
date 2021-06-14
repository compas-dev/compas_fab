"""
Calculate the robot's inverse kinematic for a given plane.

COMPAS FAB v0.18.3
"""
from ghpythonlib.componentbase import executingcomponent as component

from compas_fab.ghpython.components import coerce_frame


class InverseKinematics(component):
    def RunScript(self, robot, plane, start_configuration, group):
        configuration = None
        if robot and robot.client and robot.client.is_connected and plane:
            frame = coerce_frame(plane)
            configuration = robot.inverse_kinematics(frame, start_configuration, group)
        return configuration

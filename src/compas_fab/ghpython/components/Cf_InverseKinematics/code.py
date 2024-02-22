"""
Calculate the robot's inverse kinematic for a given plane.

COMPAS FAB v1.0.2
"""

from compas_rhino.conversions import plane_to_compas_frame
from ghpythonlib.componentbase import executingcomponent as component


class InverseKinematics(component):
    def RunScript(self, robot, plane, start_configuration, group):
        configuration = None
        if robot and robot.client and robot.client.is_connected and plane:
            frame = plane_to_compas_frame(plane)
            configuration = robot.inverse_kinematics(frame, start_configuration, group)
        return configuration

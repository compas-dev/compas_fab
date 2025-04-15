# r: compas_fab>=1.0.2
"""
Calculate the robot's inverse kinematic for a given plane.

COMPAS FAB v1.0.2
"""

import Grasshopper
from compas_rhino.conversions import plane_to_compas_frame


class InverseKinematics(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot, plane, start_configuration, group):
        configuration = None
        if robot and robot.client and robot.client.is_connected and plane:
            frame = plane_to_compas_frame(plane)
            configuration = robot.inverse_kinematics(frame, start_configuration, group)
        return configuration

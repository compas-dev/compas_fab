"""
Calculate the robot's inverse kinematic for a given plane.

COMPAS FAB v0.26.0
"""
from compas_rhino.conversions import RhinoPlane
from ghpythonlib.componentbase import executingcomponent as component


class InverseKinematics(component):
    def RunScript(self, robot, plane, start_configuration, group):
        configuration = None
        if robot and robot.client and robot.client.is_connected and plane:
            frame = RhinoPlane.from_geometry(plane).to_compas_frame()
            configuration = robot.inverse_kinematics(frame, start_configuration, group)
        return configuration

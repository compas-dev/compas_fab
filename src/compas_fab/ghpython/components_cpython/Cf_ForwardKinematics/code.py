# r: compas_fab>=1.0.2
"""
Calculate the robot's forward kinematic for a given configuration.

COMPAS FAB v1.1.0
"""

import Grasshopper


class ForwardKinematics(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot, config, group, link, use_only_model):
        frame = None
        if robot and config:
            options = {}
            if link:
                options["link"] = link
            if use_only_model:
                options["solver"] = "model"
            frame = robot.forward_kinematics(config, group, options=options)
        return frame

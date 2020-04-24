from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class RobotForwardKinematics(object):
    def __init__(self, robot):
        self.robot = robot
        self.forward_kinematics = self.robot.forward_kinematics

    def __call__(self, joint_state, link_name=None):
        return self.forward_kinematics(joint_state, link_name)

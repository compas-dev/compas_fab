from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class RobotInverseKinematics(object):
    def __init__(self, robot):
        self.robot = robot
        self.inverse_kinematics = self.robot.inverse_kinematics

    def __call__(self, frame_WCF, start_configuration=None,
                 group=None, avoid_collisions=True,
                 constraints=None, attempts=8,
                 attached_collision_meshes=None,
                 return_full_configuration=False):
        return self.inverse_kinematics(frame_WCF, start_configuration,
                                       group, avoid_collisions,
                                       constraints, attempts,
                                       attached_collision_meshes,
                                       return_full_configuration)

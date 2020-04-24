from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class RobotPlanCartesianMotion(object):
    def __init__(self, robot):
        self.robot = robot
        self.plan_cartesian_motion = self.robot.plan_cartesian_motion

    def __call__(self, frames_WCF, start_configuration=None,
                 max_step=0.01, jump_threshold=1.57,
                 avoid_collisions=True, group=None,
                 path_constraints=None,
                 attached_collision_meshes=None):
        return self.plan_cartesian_motion(frames_WCF, start_configuration,
                                          max_step, jump_threshold,
                                          avoid_collisions, group,
                                          path_constraints,
                                          attached_collision_meshes)

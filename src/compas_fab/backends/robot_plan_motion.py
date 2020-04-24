from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class RobotPlanMotion(object):
    def __init__(self, robot):
        self.robot = robot
        self.plan_motion = self.robot.plan_motion

    def __call__(self, goal_constraints, start_configuration=None,
                 group=None, path_constraints=None, planner_id='RRT',
                 num_planning_attempts=1, allowed_planning_time=2.,
                 max_velocity_scaling_factor=1.,
                 max_acceleration_scaling_factor=1.,
                 attached_collision_meshes=None):
        return self.plan_motion(goal_constraints, start_configuration,
                                group, path_constraints, planner_id,
                                num_planning_attempts, allowed_planning_time,
                                max_velocity_scaling_factor,
                                max_acceleration_scaling_factor,
                                attached_collision_meshes)

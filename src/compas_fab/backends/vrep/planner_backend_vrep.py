from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.client import PlannerInterface
from compas_fab.backends.vrep.backend_features.vrep_forward_kinematics import VrepForwardKinematics
from compas_fab.backends.vrep.backend_features.vrep_inverse_kinematics import VrepInverseKinematics
from compas_fab.backends.vrep.backend_features.vrep_plan_motion import VrepPlanMotion


class VrepPlanner(PlannerInterface):
    def __init__(self, client):
        super(VrepPlanner, self).__init__(client)

    def forward_kinematics(self, robot):
        return VrepForwardKinematics(self.client)(robot)

    def inverse_kinematics(self, robot, goal_frame, metric_values=None, gantry_joint_limits=None, arm_joint_limits=None, max_trials=None, max_results=1):
        return VrepInverseKinematics(self.client)(robot, goal_frame, metric_values, gantry_joint_limits, arm_joint_limits, max_trials, max_results)

    def plan_motion(self, robot, goal_frame, metric_values=None, collision_meshes=None,
                    planner_id='rrtconnect', trials=1, resolution=0.02,
                    gantry_joint_limits=None, arm_joint_limits=None, shallow_state_search=True, optimize_path_length=False):
        return VrepPlanMotion(self.client)(robot, goal_frame, metric_values, collision_meshes,
                                           planner_id, trials, resolution,
                                           gantry_joint_limits, arm_joint_limits, shallow_state_search, optimize_path_length)

    def plan_motion_to_config(self, robot, goal_configs, metric_values=None, collision_meshes=None,
                              planner_id='rrtconnect', trials=1, resolution=0.02,
                              gantry_joint_limits=None, arm_joint_limits=None, shallow_state_search=True, optimize_path_length=False):
        return VrepPlanMotion(self.client).plan_motion(robot, goal_configs, metric_values, collision_meshes,
                                                       planner_id, trials, resolution,
                                                       gantry_joint_limits, arm_joint_limits, shallow_state_search, optimize_path_length)

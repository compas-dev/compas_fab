"""Interface definition for the choreo planer.

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.ros.plugins import PlannerPlugin

class ChoreoPlanner(PlannerPlugin):
    """Implement the planner backend interface based on choreo
    """

    def inverse_kinematics_async(self, callback, errback, frame, base_link, group,
                                 joint_names, joint_positions, avoid_collisions=True,
                                 constraints=None, attempts=8, attached_collision_meshes=None):
        raise NotImplementedError

    def forward_kinematics_async(self, callback, errback, joint_positions, base_link,
                                 group, joint_names, ee_link):
        raise NotImplementedError

    def plan_cartesian_motions_async(self, callback, errback, frames, base_link,
                                     ee_link, group, joint_names, joint_types,
                                     start_configuration, max_step, jump_threshold,
                                     avoid_collisions, path_constraints,
                                     attached_collision_meshes):
        raise NotImplementedError

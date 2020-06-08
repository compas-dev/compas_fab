from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class ClientInterface(object):
    def __init__(self):
        self.planner = PlannerInterface(self)
        # self.control = ControlInterface()

    def inverse_kinematics(self, *args, **kwargs):
        return self.planner.inverse_kinematics(*args, **kwargs)

    def forward_kinematics(self, *args, **kwargs):
        return self.planner.forward_kinematics(*args, **kwargs)

    def plan_cartesian_motion(self, *args, **kwargs):
        return self.planner.plan_cartesian_motion(*args, **kwargs)

    def plan_motion(self, *args, **kwargs):
        return self.planner.plan_motion(*args, **kwargs)

    # ==========================================================================
    # collision objects and planning scene
    # ==========================================================================

    def get_planning_scene(self, *args, **kwargs):
        return self.planner.get_planning_scene(*args, **kwargs)

    def add_collision_mesh(self, *args, **kwargs):
        return self.planner.add_collision_mesh(*args, **kwargs)

    def remove_collision_mesh(self, *args, **kwargs):
        return self.planner.remove_collision_mesh(*args, **kwargs)

    def append_collision_mesh(self, *args, **kwargs):
        return self.planner.append_collision_mesh(*args, **kwargs)

    def add_attached_collision_mesh(self, *args, **kwargs):
        return self.planner.add_attached_collision_mesh(*args, **kwargs)

    def remove_attached_collision_mesh(self, *args, **kwargs):
        return self.planner.remove_attached_collision_mesh(*args, **kwargs)

#     # ==========================================================================
#     # executing
#     # ==========================================================================
#
#     def execute_joint_trajectory(self, *args, **kwargs):
#         self.control.execute_joint_trajectory(*args, **kwargs)
#
#     def follow_joint_trajectory(self, *args, **kwargs):
#         self.control.follow_joint_trajectory(*args, **kwargs)
#
#     def get_configuration(self, *args, **kwargs):
#         self.control.get_configuration(*args, **kwargs)
#
#
# class ControlInterface(object):
#     def execute_joint_trajectory(self, *args, **kwargs):
#         raise NotImplementedError('Assigned control does not have this feature.')
#
#     def follow_joint_trajectory(self, *args, **kwargs):
#         raise NotImplementedError('Assigned control does not have this feature.')
#
#     def get_configuration(self, *args, **kwargs):
#         raise NotImplementedError('Assigned control does not have this feature.')


class PlannerInterface(object):
    def __init__(self, client):
        super(PlannerInterface, self).__init__()
        self.client = client

    # ==========================================================================
    # planning services
    # ==========================================================================

    def inverse_kinematics(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def forward_kinematics(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def plan_motion(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def plan_cartesian_motion(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    # ==========================================================================
    # collision objects and planning scene
    # ==========================================================================

    def get_planning_scene(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def add_collision_mesh(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def remove_collision_mesh(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def append_collision_mesh(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def add_attached_collision_mesh(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

    def remove_attached_collision_mesh(self, *args, **kwargs):
        raise Exception('Assigned planner does not have this feature.')

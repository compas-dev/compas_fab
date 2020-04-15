from __future__ import print_function


class ClientInterface(object):
    def __init__(self):
        self.planner = PlannerInterface(None)
        self.control = ControlInterface()

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

    def add_collision_mesh(self, collision_mesh):
        return self.planner.add_collision_mesh(collision_mesh)

    def remove_collision_mesh(self, id):
        return self.planner.remove_collision_mesh(id)

    def append_collision_mesh(self, collision_mesh):
        return self.planner.append_collision_mesh(collision_mesh)

    def add_attached_collision_mesh(self, attached_collision_mesh):
        return self.planner.add_attached_collision_mesh(attached_collision_mesh)

    def remove_attached_collision_mesh(self, id):
        return self.planner.remove_attached_collision_mesh(id)

    # ==========================================================================
    # executing
    # ==========================================================================

    def execute_joint_trajectory(self, *args, **kwargs):
        self.control.execute_joint_trajectory(*args, **kwargs)

    def follow_joint_trajectory(self, *args, **kwargs):
        self.control.follow_joint_trajectory(*args, **kwargs)

    def get_configuration(self, *args, **kwargs):
        self.control.get_configuration(*args, **kwargs)


class ControlInterface(object):
    def execute_joint_trajectory(self, *args, **kwargs):
        raise NotImplementedError('Assigned control does not have this feature.')

    def follow_joint_trajectory(self, *args, **kwargs):
        raise NotImplementedError('Assigned control does not have this feature.')

    def get_configuration(self, *args, **kwargs):
        raise NotImplementedError('Assigned control does not have this feature.')


class PlannerInterface(object):
    def __init__(self, context):
        self.context = context

    def validate_response(self, response):
        pass

    # ==========================================================================
    # planning services
    # ==========================================================================

    def inverse_kinematics(self, *args, **kwargs):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def forward_kinematics(self, *args, **kwargs):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def plan_motion(self, *args, **kwargs):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def plan_cartesian_motion(self, *args, **kwargs):
        raise NotImplementedError('Assigned planner does not have this feature.')

    # ==========================================================================
    # collision objects and planning scene
    # ==========================================================================

    def get_planning_scene(self, *args, **kwargs):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def add_collision_mesh(self, collision_mesh):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def remove_collision_mesh(self, id):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def append_collision_mesh(self, collision_mesh):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def add_attached_collision_mesh(self, attached_collision_mesh):
        raise NotImplementedError('Assigned planner does not have this feature.')

    def remove_attached_collision_mesh(self, id):
        raise NotImplementedError('Assigned planner does not have this feature.')

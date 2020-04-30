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


# class CollisionObjectListener(object):
#     def __init__(self, client):
#         self.client = client
#
#     def has_topic(self):
#         return hasattr(self.client, 'collision_object_topic') and self.client.collision_object_topic
#
#     def initialize(self):
#         if self.has_topic():
#             return
#         self.client.collision_object_topic = Topic(
#             self.client,
#             '/collision_object',
#             'moveit_msgs/CollisionObject',
#             queue_size=None)
#         self.client.collision_object_topic.advertise()
#
#     def dispose(self):
#         if self.has_topic():
#             self.client.collision_object_topic.unadvertise()
#
#     def collision_object(self, collision_object, operation=CollisionObject.ADD):
#         if not self.has_topic():
#             self.initialize()
#         collision_object.operation = operation
#         self.client.collision_object_topic(collision_object.msg)
#
#
# class AttachedCollisionObjectManager(object):
#     def __init__(self, client):
#         self.client = client
#
#     def has_topic(self):
#         return hasattr(self.client, 'attached_collision_object_topic') and self.client.attached_collision_object_topic
#
#     def initialize(self):
#         if self.has_topic():
#             return
#
#         self.client.attached_collision_object_topic = Topic(
#             self.client,
#             '/attached_collision_object',
#             'moveit_msgs/AttachedCollisionObject',
#             queue_size=None)
#         self.client.attached_collision_object_topic.advertise()
#
#     def dispose(self):
#         if self.has_topic():
#             self.client.attached_collision_object_topic.unadvertise()
#
#     def attached_collision_object(self, attached_collision_object, operation=CollitionObject.ADD):
#         if not self.has_topic():
#             self.initialize()
#         attached_collision_object.object.operation = operation
#         self.client.attached_collision_object_topic.publish(attached_collision_object.msg)
#
#
# class Subject(object):
#     def __init__(self):
#         self._listeners = []
#
#     def attach(self, listener):
#         self._listeners.append(listener)
#
#     def detach(self, listener):
#         self._listeners.remove(listener)
#
#     def notify(self, msg):
#         for listener in self._listeners:
#             listener.update(self, msg)


class PlannerInterface(object):  # inherit from Subject?
    def __init__(self, client):
        self.client = client

    def init_planner(self):
        pass

    def dispose_planner(self):
        pass

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

    def add_collision_mesh(self, collision_mesh):
        raise Exception('Assigned planner does not have this feature.')

    def remove_collision_mesh(self, id):
        raise Exception('Assigned planner does not have this feature.')

    def append_collision_mesh(self, collision_mesh):
        raise Exception('Assigned planner does not have this feature.')

    def add_attached_collision_mesh(self, attached_collision_mesh):
        raise Exception('Assigned planner does not have this feature.')

    def remove_attached_collision_mesh(self, id):
        raise Exception('Assigned planner does not have this feature.')

"""
Internal implementation of the planner backend interface for MoveIt!
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from roslibpy import Topic

from compas_fab.backends.client import PlannerInterface
from compas_fab.backends.ros.backend_features.move_it_add_attached_collision_mesh import MoveItAddAttachedCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_add_collision_mesh import MoveItAddCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_append_collision_mesh import MoveItAppendCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_forward_kinematics import MoveItForwardKinematics
from compas_fab.backends.ros.backend_features.move_it_inverse_kinematics import MoveItInverseKinematics
from compas_fab.backends.ros.backend_features.move_it_plan_cartesian_motion import MoveItPlanCartesianMotion
from compas_fab.backends.ros.backend_features.move_it_plan_motion import MoveItPlanMotion
from compas_fab.backends.ros.backend_features.move_it_planning_scene import MoveItPlanningScene
from compas_fab.backends.ros.backend_features.move_it_remove_attached_collision_mesh import MoveItRemoveAttachedCollisionMesh
from compas_fab.backends.ros.backend_features.move_it_remove_collision_mesh import MoveItRemoveCollisionMesh
from compas_fab.backends.ros.messages import CollisionObject


class MoveItPlanner(PlannerInterface):
    """Implement the planner backend interface based on MoveIt!
    """

    def __init__(self, client):
        super(MoveItPlanner, self).__init__(client)
        self.inverse_kinematics = MoveItInverseKinematics(self.client)
        self.forward_kinematics = MoveItForwardKinematics(self.client)
        self.plan_cartesian_motion = MoveItPlanCartesianMotion(self.client)
        self.plan_motion = MoveItPlanMotion(self.client)
        self.get_planning_scene = MoveItPlanningScene(self.client)
        self.add_collision_mesh = MoveItAddCollisionMesh(self.client)
        self.remove_collision_mesh = MoveItRemoveCollisionMesh(self.client)
        self.append_collision_mesh = MoveItAppendCollisionMesh(self.client)
        self.add_attached_collision_mesh = MoveItAddAttachedCollisionMesh(self.client)
        self.remove_attached_collision_mesh = MoveItRemoveAttachedCollisionMesh(self.client)

        self.on('initialize', self.init_planner)
        self.on('collision_object', self.advertise_collision_object)
        self.on('collision_object', self._collision_object)
        self.on('attached_collision_object', self.advertise_attached_collision_object)
        self.on('attached_collision_object', self._attached_collision_object)
        self.on('dispose', self.dispose_planner)

    def has_topic(self, topic):
        if hasattr(self.client, topic) and getattr(self.client, topic):
            return True
        return False

    def advertise_collision_object(self, *args, **kwargs):
        if not self.has_topic('collision_object_topic'):
            self.client.collision_object_topic = Topic(
                self.client,
                '/collision_object',
                'moveit_msgs/CollisionObject',
                queue_size=None)
            self.client.collision_object_topic.advertise()

    def advertise_attached_collision_object(self, *args, **kwargs):
        if not self.has_topic('attached_collision_object_topic'):
            self.client.attached_collision_object_topic = Topic(
                self.client,
                '/attached_collision_object',
                'moveit_msgs/AttachedCollisionObject',
                queue_size=None)
            self.client.attached_collision_object_topic.advertise()

    def init_planner(self, *args, **kwargs):
        self.advertise_collision_object()
        self.advertise_attached_collision_object()

    def dispose_planner(self, *args, **kwargs):
        if self.has_topic('collision_object_topic'):
            self.client.collision_object_topic.unadvertise()
        if self.has_topic('attached_collision_object_topic'):
            self.client.attached_collision_object_topic.unadvertise()

    # ==========================================================================
    # collision objects
    # ==========================================================================

    def _collision_object(self, *args, **kwargs):
        collision_object = kwargs.get('collision_object')
        operation = kwargs.get('operations', CollisionObject.ADD)
        collision_object.operation = operation
        self.client.collision_object_topic.publish(collision_object.msg)

    def _attached_collision_object(self, *args, **kwargs):
        attached_collision_object = kwargs.get('attached_collision_object')
        operation = kwargs.get('operation', CollisionObject.ADD)
        attached_collision_object.object.operation = operation
        self.client.attached_collision_object_topic.publish(attached_collision_object.msg)

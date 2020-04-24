"""
Internal implementation of the planner backend interface for MoveIt!
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from roslibpy import Topic

from compas_fab.backends.client import PlannerInterface
from compas_fab.backends.ros.backend_features.move_it_forward_kinematics import MoveItForwardKinematics
from compas_fab.backends.ros.backend_features.move_it_inverse_kinematics import MoveItInverseKinematics
from compas_fab.backends.ros.backend_features.move_it_plan_cartesian_motion import MoveItPlanCartesianMotion
from compas_fab.backends.ros.backend_features.move_it_plan_motion import MoveItPlanMotion
from compas_fab.backends.ros.backend_features.move_it_planning_scene import MoveItPlanningScene
from compas_fab.backends.ros.messages import AttachedCollisionObject
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

    def init_planner(self):
        self.client.collision_object_topic = Topic(
            self.client,
            '/collision_object',
            'moveit_msgs/CollisionObject',
            queue_size=None)
        self.client.collision_object_topic.advertise()

        self.client.attached_collision_object_topic = Topic(
            self.client,
            '/attached_collision_object',
            'moveit_msgs/AttachedCollisionObject',
            queue_size=None)
        self.client.attached_collision_object_topic.advertise()

    def dispose_planner(self):
        if hasattr(self.client, 'collision_object_topic') and self.client.collision_object_topic:
            self.client.collision_object_topic.unadvertise()
        if hasattr(self.client, 'attached_collision_object_topic') and self.client.attached_collision_object_topic:
            self.client.attached_collision_object_topic.unadvertise()

    # ==========================================================================
    # collision objects
    # ==========================================================================

    def add_collision_mesh(self, collision_mesh):
        """Add a collision mesh to the planning scene."""
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self._collision_object(co, CollisionObject.ADD)

    def remove_collision_mesh(self, id):
        """Remove a collision mesh from the planning scene."""
        co = CollisionObject()
        co.id = id
        self._collision_object(co, CollisionObject.REMOVE)

    def append_collision_mesh(self, collision_mesh):
        """Append a collision mesh to the planning scene."""
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self._collision_object(co, CollisionObject.APPEND)

    def _collision_object(self, collision_object, operation=CollisionObject.ADD):
        if not hasattr(self, 'collision_object_topic') or not self.client.collision_object_topic:
            self.init_planner()

        collision_object.operation = operation
        self.client.collision_object_topic.publish(collision_object.msg)

    def add_attached_collision_mesh(self, attached_collision_mesh):
        """Add a collision mesh attached to the robot."""
        aco = AttachedCollisionObject.from_attached_collision_mesh(
            attached_collision_mesh)
        self._attached_collision_object(aco, operation=CollisionObject.ADD)

    def remove_attached_collision_mesh(self, id):
        """Add an attached collision mesh from the robot."""
        aco = AttachedCollisionObject()
        aco.object.id = id
        return self._attached_collision_object(aco, operation=CollisionObject.REMOVE)

    def _attached_collision_object(self, attached_collision_object, operation=CollisionObject.ADD):
        if not hasattr(self, 'attached_collision_object_topic') or not self.client.attached_collision_object_topic:
            self.init_planner()

        attached_collision_object.object.operation = operation
        self.client.attached_collision_object_topic.publish(attached_collision_object.msg)

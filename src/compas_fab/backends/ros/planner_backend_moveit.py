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

    def inverse_kinematics(self, robot, frame, group,
                           start_configuration, avoid_collisions=True,
                           constraints=None, attempts=8,
                           attached_collision_meshes=None):
        return MoveItInverseKinematics(self.client)(robot, frame, group,
                                                    start_configuration, avoid_collisions,
                                                    constraints, attempts,
                                                    attached_collision_meshes)

    def forward_kinematics(self, robot, configuration, group, ee_link):
        return MoveItForwardKinematics(self.client)(robot, configuration, group, ee_link)

    def plan_cartesian_motion(self,
                              robot, frames, start_configuration,
                              group, max_step, jump_threshold,
                              avoid_collisions, path_constraints,
                              attached_collision_meshes):
        return MoveItPlanCartesianMotion(self.client)(robot, frames, start_configuration,
                                                      group, max_step, jump_threshold,
                                                      avoid_collisions, path_constraints,
                                                      attached_collision_meshes)

    def plan_motion(self, robot, goal_constraints, start_configuration, group,
                    path_constraints=None, trajectory_constraints=None,
                    planner_id='', num_planning_attempts=8,
                    allowed_planning_time=2.,
                    max_velocity_scaling_factor=1.,
                    max_acceleration_scaling_factor=1.,
                    attached_collision_meshes=None,
                    workspace_parameters=None):
        return MoveItPlanMotion(self.client)(robot, goal_constraints, start_configuration, group,
                                             path_constraints, trajectory_constraints,
                                             planner_id, num_planning_attempts,
                                             allowed_planning_time,
                                             max_velocity_scaling_factor,
                                             max_acceleration_scaling_factor,
                                             attached_collision_meshes,
                                             workspace_parameters)

    # ==========================================================================
    # collision objects
    # ==========================================================================

    def get_planning_scene(self):
        return MoveItPlanningScene(self.client)()

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
        if not hasattr(self, 'collision_object_topic') or not self.collision_object_topic:
            self.init_planner()

        collision_object.operation = operation
        self.collision_object_topic.publish(collision_object.msg)

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
        if not hasattr(self, 'attached_collision_object_topic') or not self.attached_collision_object_topic:
            self.init_planner()

        attached_collision_object.object.operation = operation
        self.attached_collision_object_topic.publish(attached_collision_object.msg)

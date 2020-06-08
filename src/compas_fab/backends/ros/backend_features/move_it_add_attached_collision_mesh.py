from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import AddAttachedCollisionMesh
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject

__all__ = [
    'MoveItAddAttachedCollisionMesh',
]


class MoveItAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Add a collision mesh attached to the robot."""
        aco = AttachedCollisionObject.from_attached_collision_mesh(
            attached_collision_mesh)
        self.ros_client.planner.publish_attached_collision_object(attached_collision_object=aco, operation=CollisionObject.ADD)

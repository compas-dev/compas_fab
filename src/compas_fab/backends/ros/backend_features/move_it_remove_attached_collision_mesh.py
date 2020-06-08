from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import RemoveAttachedCollisionMesh
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject

__all__ = [
    'MoveItRemoveAttachedCollisionMesh',
]


class MoveItRemoveAttachedCollisionMesh(RemoveAttachedCollisionMesh):
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def remove_attached_collision_mesh(self, id, options=None):
        """Add an attached collision mesh from the robot."""
        aco = AttachedCollisionObject()
        aco.object.id = id
        self.ros_client.planner.publish_attached_collision_object(attached_collision_object=aco, operation=CollisionObject.REMOVE)

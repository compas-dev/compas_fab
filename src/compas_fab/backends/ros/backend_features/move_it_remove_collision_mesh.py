from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.backend_feature_interfaces import RemoveCollisionMesh
from compas_fab.backends.ros.messages import CollisionObject

__all__ = [
    'MoveItRemoveCollisionMesh',
]


class MoveItRemoveCollisionMesh(RemoveCollisionMesh):
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def remove_collision_mesh(self, id, options={}):
        """Remove a collision mesh from the planning scene."""
        co = CollisionObject()
        co.id = id
        self.ros_client.planner.emit('collision_object', collision_object=co, operation=CollisionObject.REMOVE)

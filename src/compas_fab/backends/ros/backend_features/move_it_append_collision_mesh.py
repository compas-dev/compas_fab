from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import AppendCollisionMesh
from compas_fab.backends.ros.messages import CollisionObject

__all__ = [
    'MoveItAppendCollisionMesh',
]


class MoveItAppendCollisionMesh(AppendCollisionMesh):
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def append_collision_mesh(self, collision_mesh, options=None):
        """Append a collision mesh to the planning scene."""
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self.ros_client.planner.publish_collision_object(collision_object=co, operation=CollisionObject.APPEND)

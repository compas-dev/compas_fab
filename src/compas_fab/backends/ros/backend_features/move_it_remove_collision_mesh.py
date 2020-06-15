from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import RemoveCollisionMesh
from compas_fab.backends.ros.messages import CollisionObject

__all__ = [
    'MoveItRemoveCollisionMesh',
]


class MoveItRemoveCollisionMesh(RemoveCollisionMesh):
    """Callable to remove a collision mesh from the planning scene."""
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def remove_collision_mesh(self, id, options=None):
        """Remove a collision mesh from the planning scene.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        co = CollisionObject()
        co.id = id
        self.ros_client.planner.publish_collision_object(collision_object=co, operation=CollisionObject.REMOVE)

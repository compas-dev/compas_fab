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
    """Callable to remove an attached collision mesh from the robot."""
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def remove_attached_collision_mesh(self, id, options=None):
        """Remove an attached collision mesh from the robot.

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
        aco = AttachedCollisionObject()
        aco.object.id = id
        self.ros_client.planner.publish_attached_collision_object(attached_collision_object=aco, operation=CollisionObject.REMOVE)

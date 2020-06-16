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
    """Callable to add a collision mesh and attach it to the robot."""
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Add a collision mesh and attach it to the robot.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
            Object containing the collision mesh to be attached.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        aco = AttachedCollisionObject.from_attached_collision_mesh(
            attached_collision_mesh)
        self.ros_client.planner.publish_attached_collision_object(attached_collision_object=aco, operation=CollisionObject.ADD)

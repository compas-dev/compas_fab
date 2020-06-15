from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import AddCollisionMesh
from compas_fab.backends.ros.messages import CollisionObject

__all__ = [
    'MoveItAddCollisionMesh',
]


class MoveItAddCollisionMesh(AddCollisionMesh):
    """Callable to add a collision mesh to the planning scene."""
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def add_collision_mesh(self, collision_mesh, options=None):
        """Add a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            Object containing the collision mesh to be added.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self.ros_client.planner.publish_collision_object(collision_object=co, operation=CollisionObject.ADD)

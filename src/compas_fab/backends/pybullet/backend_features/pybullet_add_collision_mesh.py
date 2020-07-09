from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import AddCollisionMesh

__all__ = [
    'PyBulletAddCollisionMesh',
]


class PyBulletAddCollisionMesh(AddCollisionMesh):
    """Callable to add a collision mesh to the planning scene."""
    def __init__(self, client):
        self.client = client

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
        mesh = collision_mesh.mesh
        name = collision_mesh.id
        frame = collision_mesh.frame

        # mimic ROS' behavior: collision object with same name is replaced
        if name in self.client.collision_objects:
            self.client.remove_collision_mesh(name)

        body_id = self.client.convert_mesh_to_body(mesh, frame, name)
        self.client.collision_objects[name] = [body_id]

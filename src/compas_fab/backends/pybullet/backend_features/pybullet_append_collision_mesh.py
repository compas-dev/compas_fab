from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import AppendCollisionMesh

__all__ = [
    'PyBulletAppendCollisionMesh',
]


class PyBulletAppendCollisionMesh(AppendCollisionMesh):
    """Callable to append a collision mesh to the planning scene."""
    def __init__(self, client):
        self.client = client

    def append_collision_mesh(self, collision_mesh, options=None):
        """Append a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            Object containing the collision mesh to be appended.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        mesh = collision_mesh.mesh
        name = collision_mesh.id
        frame = collision_mesh.frame
        if name in self.client.collision_objects:
            body_id = self.client.convert_mesh_to_body(mesh, frame, name)
            self.client.collision_objects[name].append(body_id)
        else:
            self.client.add_collision_mesh(collision_mesh)

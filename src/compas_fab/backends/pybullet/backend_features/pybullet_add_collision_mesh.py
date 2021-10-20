from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import AddCollisionMesh

__all__ = [
    'PyBulletAddCollisionMesh',
]

from compas_fab.backends.pybullet.const import STATIC_MASS


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
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"mass"``: (:obj:`float`) The mass of the object, in kg.
              If `0` is given, (the default), the object added is static.
            - ``"concavity"``: (:obj:`bool`) When ``False`` (the default),
              the mesh will be loaded as its convex hull for collision checking purposes.
              When ``True``, a non-static mesh will be decomposed into convex parts using v-HACD.

        Returns
        -------
        ``None``
        """
        options = options or {}
        mesh = collision_mesh.mesh
        name = collision_mesh.id
        frame = collision_mesh.frame
        mass = options.get('mass', STATIC_MASS)
        concavity = options.get('concavity', False)

        # mimic ROS' behavior: collision object with same name is replaced
        if name in self.client.collision_objects:
            self.client.remove_collision_mesh(name)

        body_id = self.client.convert_mesh_to_body(mesh, frame, name, concavity, mass)
        self.client.collision_objects[name] = [body_id]

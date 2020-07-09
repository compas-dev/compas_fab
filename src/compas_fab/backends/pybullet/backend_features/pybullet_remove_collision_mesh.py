from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import RemoveCollisionMesh
from compas_fab.backends.pybullet.utils import LOG
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader('pybullet', globals(), 'pybullet')


__all__ = [
    'PyBulletRemoveCollisionMesh',
]


class PyBulletRemoveCollisionMesh(RemoveCollisionMesh):
    """Callable to remove a collision mesh from the planning scene."""
    def __init__(self, client):
        self.client = client

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
        if id not in self.client.collision_objects:
            LOG.warning("Collision object with name '{}' does not exist in scene.".format(id))
            return

        for body_id in self.client.collision_objects[id]:
            pybullet.removeBody(body_id)

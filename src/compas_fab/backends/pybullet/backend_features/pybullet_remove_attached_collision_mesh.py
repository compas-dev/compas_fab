from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import RemoveAttachedCollisionMesh
from compas_fab.backends.pybullet.utils import LOG
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader('pybullet', globals(), 'pybullet')


__all__ = [
    'PyBulletRemoveAttachedCollisionMesh',
]


class PyBulletRemoveAttachedCollisionMesh(RemoveAttachedCollisionMesh):
    """Callable to remove an attached collision mesh from the robot."""
    def __init__(self, client):
        self.client = client

    def remove_attached_collision_mesh(self, id, options=None):
        """Remove an attached collision mesh from the robot.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"robot"``: (:class:`compas_fab.robots.Robot``) Robot instance
              to which the object should be attached.

        Returns
        -------
        ``None``
        """
        robot = options['robot']
        attached_collision_meshes = robot.attributes['attached_collision_meshes']
        if id not in attached_collision_meshes:
            LOG.warning("Attached collision object with name '{}' does not exist in scene.".format(id))
            return

        del attached_collision_meshes[id]

        self.client.reconstruct_robot(robot)

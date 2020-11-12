from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import AddAttachedCollisionMesh


__all__ = [
    'PyBulletAddAttachedCollisionMesh',
]


class PyBulletAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    """Callable to add a collision mesh and attach it to the robot."""

    def __init__(self, client):
        self.client = client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Add a collision mesh and attach it to the robot.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
            Object containing the collision mesh to be attached.
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"robot"``: (:class:`compas_fab.robots.Robot``) Robot instance
              to which the object should be attached.
            - ``"max_force"``: (:obj:`float`) The maximum force that
              the constraint can apply. Optional.
            - ``"mass"``: (:obj:`float`) The mass of the object, in kg.
              Defaults to ``1.0``.

        Returns
        -------
        ``None``
        """
        options = options or {}
        robot = options['robot']

        if robot.attributes.get('cached_pybullet_robot') is None:
            raise Exception('Robot must be cached before adding attached collision meshes.')

        attached_collision_mesh.attr = {'pybullet': {}}
        attached_collision_mesh.attr['pybullet']['mass'] = options.get('mass', 1.0)

        attached_collision_meshes = robot.attributes['attached_collision_meshes']
        attached_collision_meshes[attached_collision_mesh.collision_mesh.id] = attached_collision_mesh

        self.client.reconstruct_robot(robot)

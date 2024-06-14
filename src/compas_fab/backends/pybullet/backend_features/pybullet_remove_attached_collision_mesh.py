from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import RemoveAttachedCollisionMesh
from compas_fab.utilities import LazyLoader

pybullet = LazyLoader("pybullet", globals(), "pybullet")


__all__ = [
    "PyBulletRemoveAttachedCollisionMesh",
]


class PyBulletRemoveAttachedCollisionMesh(RemoveAttachedCollisionMesh):
    """Callable to remove an attached collision mesh from the robot."""

    def remove_attached_collision_mesh(self, id, options=None):
        """Remove an attached collision mesh from the robot.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict
            Dictionary containing the following key-value pairs:

            - ``"robot"``: (:class:`compas_fab.robots.Robot`) Robot instance
              to which the object should be attached.

        Returns
        -------
        ``None``
        """
        robot = options["robot"]
        self.client.ensure_cached_robot_geometry(robot)

        cached_robot_model = self.client.get_cached_robot(robot)

        # remove link and fixed joint
        cached_robot_model.remove_link(id)
        cached_robot_model.remove_joint(id + "_fixed_joint")

        self.client.reload_from_cache(robot)

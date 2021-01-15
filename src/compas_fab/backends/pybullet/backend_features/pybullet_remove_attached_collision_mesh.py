from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import RemoveAttachedCollisionMesh
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

            - ``"robot"``: (:class:`compas_fab.robots.Robot`) Robot instance
              to which the object should be attached.

        Returns
        -------
        ``None``
        """
        robot = options['robot']
        self.client.ensure_cached_robot(robot)

        current_configuration = self.client.get_robot_configuration(robot)

        cached_robot_model = robot.attributes['pybullet']['cached_robot']
        cached_robot_filepath = robot.attributes['pybullet']['cached_robot_filepath']

        # remove link and fixed joint
        cached_robot_model.remove_link(id)
        cached_robot_model.remove_joint(id + '_fixed_joint')

        robot_uid = cached_robot_model.attr['uid']
        pybullet.removeBody(robot_uid, physicsClientId=self.client.client_id)

        cached_robot_model.to_urdf_file(cached_robot_filepath, prettify=True)
        pybullet.setPhysicsEngineParameter(enableFileCaching=0)
        self.client._load_robot_to_pybullet(cached_robot_filepath, robot)
        pybullet.setPhysicsEngineParameter(enableFileCaching=1)

        self.client.set_robot_configuration(robot, current_configuration)
        self.client.step_simulation()

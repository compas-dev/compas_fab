from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import ForwardKinematics


class PyBulletForwardKinematics(ForwardKinematics):
    """Callable to calculate the robot's forward kinematic."""
    def __init__(self, client):
        self.client = client

    def forward_kinematics(self, configuration, group=None, options=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
        group : str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"robot"``: (:class:`compas_fab.robots.Robot`) The robot whose
              kinematics are to be calculated.
            - ``"link"``: (:obj:`str`, optional) The name of the link to
              calculate the forward kinematics for. Defaults to the end effector.
            - ``"check_collision"``: (:obj:`str`, optional) When ``True``,
              :meth:`compas.pybullet.PyBulletClient.check_collisions` will be called.
              Defaults to ``False``.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).
        """
        robot = options['robot']
        # Joint names, but not link names, are recoverable from the pybullet server.
        # My intention is that a user doesn't have to care about pybullet's internal id system,
        # except for maybe the robot's uid.  I would put just the robot uid in the options
        # (rather than the full robot)
        # if it were possible to recover the links, but it's not.
        link_name = options.get('link') or robot.get_end_effector_link_name(group)
        link_id = self.client._get_link_id_by_name(link_name, robot)
        self.client.set_robot_configuration(robot, configuration, group)
        frame = self.client._get_link_frame(link_id, robot.pybullet_uid)
        if options.get('check_collision'):
            self.client.collision_check()
        return frame

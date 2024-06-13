from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import ForwardKinematics


class PyBulletForwardKinematics(ForwardKinematics):
    """Callable to calculate the robot's forward kinematic."""

    def forward_kinematics(self, robot, configuration, group=None, options=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        configuration : :class:`compas_fab.robots.Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
        group : str, optional
            The planning group used for determining the end effector and labeling
            the ``configuration``. Defaults to the robot's main planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"link"``: (:obj:`str`, optional) The name of the link to
              calculate the forward kinematics for. Defaults to the end effector.
            - ``"check_collision"``: (:obj:`str`, optional) When ``True``,
              :meth:`compas_fab.backends.PyBulletClient.check_collisions` will be called.
              Defaults to ``False``.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).
        """
        options = options or {"link": None, "check_collision": False}

        link_name = options.get("link") or robot.get_end_effector_link_name(group)
        cached_robot_model = self.client.get_cached_robot_model(robot)
        body_id = self.client.get_uid(cached_robot_model)
        link_id = self.client._get_link_id_by_name(link_name, cached_robot_model)
        self.client.set_robot_configuration(robot, configuration, group)
        frame = self.client._get_link_frame(link_id, body_id)
        if options.get("check_collision"):
            self.client.check_collisions(robot, configuration)
        return frame

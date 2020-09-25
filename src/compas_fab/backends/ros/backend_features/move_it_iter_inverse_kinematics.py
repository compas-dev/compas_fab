from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends import BackendError
from compas_fab.backends.interfaces import IterInverseKinematics


class MoveItIterInverseKinematics(IterInverseKinematics):
    """Callable to iterate calculation of the robot's inverse kinematics for a given frame."""
    def __init__(self, ros_client):
        self.ros_client = ros_client

    def iter_inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Iterate the calculation of the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the zero configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
              Defaults to the model's root link.
            - ``"avoid_collisions"``: (:obj:`bool`, optional) Whether or not to avoid collisions.
              Defaults to ``True``.
            - ``"constraints"``: (:obj:`list` of :class:`compas_fab.robots.Constraint`, optional)
              A set of constraints that the request must obey.  Constraints may be leveraged
              to yield a diversity of inverse kinematic solutions.
              Defaults to ``None``.
            - ``"attempts"``: (:obj:`int`, optional) The maximum number of inverse kinematic attempts.
              Defaults to ``8``.
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`, optional)
              Defaults to ``None``.
            - ``"use_random_configuration"``: (:obj:`bool`, optional): If ``True``,  a random start
              configuration is used in the inverse kinematic calculation. Defaults to ``False``.

        Raises
        ------
        compas_fab.backends.exceptions.BackendError
            If no configuration can be found.

        Yields
        -------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.
        """
        # TODO an example
        options = options or {}
        group = group or robot.main_group_name

        while True:
            if options.get('use_random_configuration'):
                start_configuration = robot.random_configuration(group=group)

            try:
                yield self.ros_client.inverse_kinematics(robot,
                                                         frame_WCF,
                                                         start_configuration=start_configuration,
                                                         group=group,
                                                         options=options)
            except BackendError:
                yield "hello"  # TODO what should happen here? continue/break/yield None?

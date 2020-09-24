from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.geometry import Sphere

from compas_fab.backends import BackendError
from compas_fab.backends.interfaces import IterInverseKinematics
from compas_fab.robots.constraints import PositionConstraint


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
              A set of constraints that the request must obey.
              Defaults to ``None``.
            - ``"attempts"``: (:obj:`int`, optional) The maximum number of inverse kinematic attempts.
              Defaults to ``8``.
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`, optional)
              Defaults to ``None``.
            - ``"max_reach"``: (:obj:`float`, optional): TODO
            - ``"link_name:``: (:obj:`str`, optional): TODO

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
        max_reach = options.get('max_reach')  # TODO default value?
        link_name = options.get('link_name')  # TODO or default value here?
        constraints = options.setdefault('constraints', [])

        group = group or robot.main_group_name

        while True:

            if max_reach and link_name:
                # forcing diverse IK solutions by constraining the link_name within a sphere
                #
                # here I just want to bring up a discussion point:  since inverse_kinematics is called, and one might
                # expect there to be constraints already in options, does it really make sense to add these two new
                # parameters of link_name and max_reach?  maybe it would be more general if the user would feel free
                # to generate his own constraints, and there could be a flag in the options on whether to generate
                # random start configurations.
                assert(link_name in robot.get_link_names(group=group))
                sphere = Sphere(frame_WCF.point, max_reach)
                constraints.append(PositionConstraint.from_sphere(link_name, sphere))
            else:
                # forcing diverse IK solutions by randomizing the start configuration
                start_configuration = robot.random_configuration(group=group)

            try:
                yield self.ros_client.inverse_kinematics(robot,
                                                         frame_WCF,
                                                         start_configuration=start_configuration,
                                                         group=group,
                                                         options=options)
            except BackendError:
                yield "hello"  # TODO what should happen here? continue/break/yield None?

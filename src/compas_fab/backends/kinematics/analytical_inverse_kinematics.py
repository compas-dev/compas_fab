from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.interfaces import InverseKinematics
from .utils import try_to_fit_configurations_between_bounds
from .utils import joint_angles_to_configurations
from .exceptions import InverseKinematicsError
from .solvers import PLANNER_BACKENDS


class AnalyticalInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame.

    Parameters
    ----------
    client : :class:`compas_fab.backends.interfaces.ClientInterface`, optional
        The backend client to use for communication, for now only the
        :class:`compas_fab.backends.PyBulletClient` is supported.
    solver : :obj:`str`, optional
        The solver to use to calculate IK.

    Notes
    -----
    This works only for industrial robot arms with six revolute joints and only
    with a client that supports ``"check_collision"``, so for now only the
    `PyBulletClient`.
    """

    def __init__(self, client=None, solver=None):
        self.client = client
        self.planner = PLANNER_BACKENDS[solver]() if solver else None

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic (IK) for a given frame.

        The IK for 6-axis industrial robots returns by default 8 possible solutions.
        These solutions have an order. That means that if you call IK on two
        subsequent frames and compare the 8 configurations of the first frame
        with the 8 configurations of the second frame at their respective indices,
        then these configurations are "close" to one another. For this reason,
        for certain use cases, e.g. for cartesian path planning, it makes sense
        to keep the order of solutions. This can be achieved by setting the
        optional parameter ``keep_order`` to ``True``. The configurations that
        are in collision or outside joint boundaries are then not removed from
        the list of solutions, they are set to ``None``.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The start_configuration of the robot.
        group: str, optional
            The planning group used for determining the end effector and labeling
            the ``start_configuration``. Defaults to the robot's main planning group.
        options: dict
            Dictionary containing the following key-value pairs:
            - ``"solver"``: (:obj:`str`) The solver to use to calculate IK.
            - ``"check_collision"``: (:obj:`str`, optional ) When ``True``, checks
                if the robot is in collision. Defaults to ``False``.
            - ``"keep_order"``: (:obj:`str`, optional ) When ``False``, removes the
                ``None``- solutions. Defaults to ``False``.

        Yields
        -------
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list
            of matching joint names. If ``"keep_order"`` is ``True`` this list
            contains also ``None``, ``None``

        Notes
        -----
        This will only work with robots that have 6 revolute joints.

        Raises
        ------
        ValueError : If the solver to solve the kinematics has not been passed.
        """

        options = options or {}
        solver = options.get('solver')
        if solver:
            self.planner = PLANNER_BACKENDS[solver]()
        elif not self.planner:  # no solver, no planner
            raise ValueError("Please pass an inverse kinematics solver")

        keep_order = options.get("keep_order", False)

        # convert the frame WCF to RCF
        base_frame = robot.get_base_frame(group=group, full_configuration=start_configuration)
        frame_RCF = base_frame.to_local_coordinates(frame_WCF)

        # calculate inverse with 8 solutions
        try:
            solutions = self.planner.inverse(frame_RCF)
        except ValueError:
            raise InverseKinematicsError()
        configurations = joint_angles_to_configurations(robot, solutions, group=group)

        # check collisions for all configurations (>> sets those to `None` that are not working)
        if options.get("check_collision", False) is True:
            acms = options.get('attached_collision_meshes', [])
            for acm in acms:
                cached_robot_model = self.client.get_cached_robot(robot)
                if not cached_robot_model.get_link_by_name(acm.collision_mesh.id):
                    self.client.add_attached_collision_mesh(acm, options={'robot': robot})
                    for touch_link in acm.touch_links:
                        self.client.disabled_collisions.add((touch_link, acm.collision_mesh.id))
            for i, config in enumerate(configurations):
                try:
                    self.client.check_collisions(robot, config)
                except BackendError:
                    configurations[i] = None

        # fit configurations within joint bounds (>> sets those to `None` that are not working)
        configurations = try_to_fit_configurations_between_bounds(robot, configurations, group=group)

        if not any(configurations):
            raise InverseKinematicsError()

        for config in configurations:
            if config:
                yield config.joint_values, config.joint_names
            elif keep_order:
                yield None, None

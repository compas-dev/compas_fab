from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Generator  # noqa: F401
        from typing import Dict  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401

        from compas_fab.backends import MoveItPlanner  # noqa: F401
        from compas_fab.backends import RosClient  # noqa: F401

from compas.tolerance import TOL
from compas.utilities import await_callback

from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import GetPositionIKRequest
from compas_fab.backends.ros.messages import GetPositionIKResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.messages import PoseStamped
from compas_fab.backends.ros.messages import PositionIKRequest
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages import RosDistro
from compas_fab.backends.ros.service_description import ServiceDescription
from compas_fab.robots import FrameTarget
from compas_fab.backends.ros.exceptions import RosValidationError

__all__ = [
    "MoveItInverseKinematics",
]


class MoveItInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""

    def __init__(self):
        # The following fields are used to store the last ik request for iterative calls
        self._last_ik_request = {"request_id": None, "solutions": None}
        # Initialize the super class
        super(MoveItInverseKinematics, self).__init__()

    GET_POSITION_IK = ServiceDescription(
        "/compute_ik", "GetPositionIK", GetPositionIKRequest, GetPositionIKResponse, validate_response
    )

    def inverse_kinematics(self, target, robot_cell_state, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Configuration
        """Calculate the robot's inverse kinematic for a given target.

        The actual implementation can be found in the :meth:`iter_inverse_kinematics` method.
        Calling `inverse_kinematics()` will return the first solution found by the iterator,
        subsequent calls will return the next solution from the iterator. Once
        all solutions have been exhausted, the iterator will be re-initialized.

        MoveIt's inverse kinematics solver accepts FrameTarget as input.
        (PointAxisTarget is not yet supported)

        For target-specific implementation details, see
        :meth:`_iter_inverse_kinematics_frame_target` for
        :class:`compas_fab.robots.FrameTarget`.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        target : :class:`compas_fab.robots.FrameTarget` or :class:`compas_fab.robots.PointAxisTarget`
            The target to calculate the inverse kinematics for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The starting state to calculate the inverse kinematics for.
            The robot's configuration in the scene is taken as the starting configuration.
        group : str, optional
            The planning group used for calculation.
            Defaults to the robot's main planning group.
        options : dict, optional
            Dictionary containing arguments specific to the solver.
            See the `options` parameter in :meth:`iter_inverse_kinematics` for details.

        Returns
        -------
        :obj:`compas_robots.Configuration`
            One of the possible IK configurations that reaches the target.

        Raises
        ------

        :class:`compas_fab.backends.exceptions.InverseKinematicsError`
            If no configuration can be found.

        :class:`compas_fab.backends.exceptions.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.


        """
        # Set default group name
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        group = group or client.robot_cell.main_group_name

        # The caching mechanism is implemented in the iter_inverse_kinematics method
        # located in InverseKinematics class. This method is just a wrapper around it
        # so that Intellisense and Docs can point here.
        configuration = super(MoveItInverseKinematics, self).inverse_kinematics(
            target, robot_cell_state, group, options
        )

        # After the caching, it calls the iter_inverse_kinematics method below.

        return configuration

    def iter_inverse_kinematics(self, target, robot_cell_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given target.

        The MoveIt inverse kinematics solver make use of the IK solver pre-configured in
        the MoveIt config file.

        This particular function wraps the MoveIt IK solver to provide a generator
        that can yield multiple IK solutions. The solver will make multiple attempts
        to find a solution. The first attempt will start from the robot's current configuration
        provided in the ``robot_cell_state``. The subsequent attempts will start from a random
        configuration, so the results may vary between calls.


        Parameters
        ----------
        target: :class:`compas.geometry.FrameTarget`
            The Frame Target to calculate the inverse for.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The starting state to calculate the inverse kinematics for.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"max_results"``: (:obj:`int`) Maximum number of results to return.
              If set to 1, the solver will be deterministic, descending from the initial
              robot configuration.
              Defaults to ``100``.
            - ``"solution_uniqueness_threshold_prismatic"``: (:obj:`float`, optional) The minimum
              distance between two solutions in the prismatic joint space to consider them unique.
              Units are in meters. Defaults to ``3e-4``.
            - ``"solution_uniqueness_threshold_revolute"``: (:obj:`float`, optional) The minimum
              distance between two solutions in the revolute joint space to consider them unique.
              Units are in radians. Defaults to ``1e-3``.
            - ``"return_full_configuration"``: (:obj:`bool`, optional)
                Whether or not to return the full configuration. Defaults to ``False``.
            - ``"verbose"``: (:obj:`bool`, optional)
                Whether or not to print verbose output. Defaults to ``False``.
            - ``"allow_collision"``: (:obj:`bool`, optional) When True, collision checking is disabled.
              Defaults to ``False``.
            - ``"constraints"``: (:obj:`list` of :class:`compas_fab.robots.Constraint`, optional)
              An extra set of MoveIt constraints where the final link of the planning group must satisfy.
              Defaults to ``None``.
            - ``"timeout"``: (:obj:`int`, optional) Maximum allowed time for one inverse kinematic
              calculation by the backend. If ``max_results`` is greater than 1, this timeout is
              applied to each calculation.
              Unit is seconds. Defaults to ``2``.
              This value supersedes the ``"attempts"`` argument used before ROS Noetic.

        Raises
        ------

        :class:`compas_fab.backends.InverseKinematicsError`
            Indicates that no IK solution could be found by the kinematic solver
            after the maximum number of attempts (``max_results``). This can be caused by
            reachability or collision or both.

        :class:`compas_fab.backends.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.

        :class:`compas_fab.backends.CollisionCheckError`
            If ``check_collision`` is enabled and the configuration is in collision.
            This is only raised if ``max_results`` is set to 1. In this case, the solver is
            deterministic (descending from the initial robot configuration) and this error
            indicates that the problem is caused by collision and not because of reachability.

        Yields
        ------
        :obj:`compas_robots.Configuration`
            One of the possible IK configurations that reaches the target.

        """
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell  # type: RobotCell
        group = group or robot_cell.main_group_name

        # Calling the super class method, which contains input sanity checks and scale normalization
        # Those are common to all planners and should be called first.
        super(MoveItInverseKinematics, self).iter_inverse_kinematics(target, robot_cell_state, group, options)

        # ===================================================================================
        # Different target types have different implementations
        # ===================================================================================

        if isinstance(target, FrameTarget):
            ik_generator = self._iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
        else:
            raise NotImplementedError("{} is not supported by MoveItInverseKinematics".format(type(target)))

        # ===================================================================================
        # Check output before yielding
        # ===================================================================================

        for configuration in ik_generator:
            # Insert any checks needed here. No checks at the moment.
            yield configuration

    def _iter_inverse_kinematics_frame_target(self, target, robot_cell_state, group, options=None):
        # type: (FrameTarget, RobotCellState, str, Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given frame target.

        This function is not exposed to the user and therefore docstrings
        should be written in iter_inverse_kinematics.

        """

        # Housekeeping for intellisense
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell  # type: RobotCell

        # NOTE: group is not optional in this inner function.
        if group not in robot_cell.robot_semantics.groups:
            raise ValueError("Planning group '{}' not found in the robot's semantics.".format(group))

        # Default options
        options = options or {}
        options["max_results"] = options.get("max_results", 100)

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", robot_cell.root_name)
        # The exposed "base_link" option is removed from documentation because I don't know what it does.

        # Setting the entire robot cell state, including the robot configuration
        planner.set_robot_cell_state(robot_cell_state)

        # Transform the Target.target_frame to Planner Coordinate Frame depending on target.target_mode
        target_pcf = client.robot_cell.target_frames_to_pcf(
            robot_cell_state, target.target_frame, target.target_mode, group
        )

        # ===================================================================================
        # Formatting input for ROS MoveIt
        # ===================================================================================

        # Start configuration
        start_configuration = robot_cell_state.robot_configuration
        # Merge with zero configuration ensure all joints are present
        start_configuration = robot_cell.zero_full_configuration().merged(start_configuration)

        # Compose the kwargs for the async function
        kwargs = {}
        kwargs["options"] = options
        kwargs["frame_WCF"] = target_pcf
        kwargs["group"] = group
        kwargs["start_configuration"] = start_configuration
        kwargs["errback_name"] = "errback"

        max_results = options.get("max_results")
        if max_results < 1:
            raise ValueError("max_results must be greater than 0.")
        all_yielded_joint_positions = []

        # Loop to get multiple results with random restarts
        for i in range(max_results):

            # First iteration uses the provided start_configuration
            # Subsequent iterations use a random configuration

            # Try block to catch backend level exceptions
            try:
                joint_positions, joint_names = await_callback(self._inverse_kinematics_async, **kwargs)
            except RosValidationError as e:
                if options.get("verbose", False):
                    print("Failed to find a configuration in iteration {} / {}".format(i, max_results))
                raise e.original_exception

            # TODO: Implement solution_uniqueness_threshold_prismatic and solution_uniqueness_threshold_revolute
            # Check if the result is the same as any of the previous results
            result_is_unique = True
            for j in all_yielded_joint_positions:
                if TOL.is_allclose(joint_positions, j, rtol=TOL.APPROXIMATION):
                    result_is_unique = False

            # If result is unique, we yield it back to the user
            if result_is_unique:
                return_full_configuration = options.get("return_full_configuration", False)
                configuration = planner._build_configuration(
                    joint_positions, joint_names, group, return_full_configuration, start_configuration
                )
                yield configuration
                all_yielded_joint_positions.append(joint_positions)

            # Generate random start configuration for next iteration
            random_configuration = robot_cell.random_configuration(kwargs["group"])
            kwargs["start_configuration"] = random_configuration

        # If no solution was found after max_results, raise an error
        if len(all_yielded_joint_positions) == 0:
            raise InverseKinematicsError(
                "No solution found after {} attempts (max_results).".format(options.get("max_results")),
                target_pcf=target_pcf,
            )

    def _inverse_kinematics_async(
        self, callback, errback, frame_WCF, start_configuration=None, group=None, options=None
    ):
        """Asynchronous handler of MoveIt IK service."""
        base_link = options["base_link"]
        header = Header(frame_id=base_link)
        pose_stamped = PoseStamped(header, Pose.from_frame(frame_WCF))

        joint_state = JointState(
            name=start_configuration.joint_names, position=start_configuration.joint_values, header=header
        )

        # The start state being in diff mode allows it to keep previously set Attached Collision Objects
        robot_state = RobotState(joint_state, MultiDOFJointState(header=header), is_diff=True)

        # Filter needs to happen after all objects have been added
        robot_state.filter_fields_for_distro(self.client.ros_distro)

        constraints = convert_constraints_to_rosmsg(options.get("constraints"), header)

        timeout_in_secs = options.get("timeout", 2)
        timeout_duration = {"secs": timeout_in_secs, "nsecs": 0}

        ik_request = PositionIKRequest(
            group_name=group,
            robot_state=robot_state,
            constraints=constraints,
            pose_stamped=pose_stamped,
            avoid_collisions=not options.get("allow_collision", False),
            attempts=options.get("attempts", 8),
            timeout=timeout_duration,
        )

        # The field `attempts` was removed in Noetic (and higher)
        # so it needs to be removed from the message otherwise it causes a serialization error
        # https://github.com/ros-planning/moveit/pull/1288
        if self.client.ros_distro not in (RosDistro.KINETIC, RosDistro.MELODIC):
            del ik_request.attempts

        def convert_to_positions(response):
            callback((response.solution.joint_state.position, response.solution.joint_state.name))

        self.GET_POSITION_IK(self.client, (ik_request,), convert_to_positions, errback)

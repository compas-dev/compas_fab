from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from copy import deepcopy

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Generator  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401
        from typing import Dict  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import FrameTarget  # noqa: F401
        from compas_fab.backends import MoveItPlanner  # noqa: F401
        from compas_fab.backends import RosClient  # noqa: F401

from compas.tolerance import TOL
from compas.utilities import await_callback

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

        Pybullet's inverse kinematics solver accepts FrameTarget and PointAxisTarget as input.
        The planner is a gradient descent solver, the initial position of the robot
        (supplied in the robot_cell_state) affects the first search attempt.
        Subsequent attempts will start from a random configuration, so the results may vary.

        For target-specific implementation details, see
        :meth:`iter_inverse_kinematics_frame_target` for
        :class:`compas_fab.robots.FrameTarget` and
        :meth:`iter_inverse_kinematics_point_axis_target` for
        :class:`compas_fab.robots.PointAxisTarget`.

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
            Dictionary containing kwargs for arguments specific to
            the underlying function being called.
            See the target-specific function's documentation for details.

        Raises
        ------
        :class: `compas_fab.backends.exceptions.InverseKinematicsError`
            If no configuration can be found.

        :class:`compas_fab.backends.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.

        Returns
        -------
        :obj:`compas_robots.Configuration`
            The calculated configuration.

        """
        # Set default group name
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        group = group or client.robot_cell.main_group_name

        # Make a copy of the options because we will modify it
        # Note: Modifying the options dict accidentally will break the hashing function in the inverse_kinematics()
        options = deepcopy(options) if options else {}
        start_configuration = deepcopy(robot_cell_state.robot_configuration)
        robot_cell_state = deepcopy(robot_cell_state)

        # The caching mechanism is implemented in the iter_inverse_kinematics method
        # located in InverseKinematics class. This method is just a wrapper around it
        # so that Intellisense and Docs can point here.
        configuration = super(PyBulletInverseKinematics, self).inverse_kinematics(
            target, robot_cell_state, group, options
        )

        # NOTE: The following check is a workaround to detect planning group that are not supported by PyBullet.
        #       In those cases, some joints outside of the group will be changed inadvertently.

        self._check_configuration_match_group(start_configuration, configuration, group)
        return configuration

    def iter_inverse_kinematics(self, target, start_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Generator[Configuration | None]
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        target: :class:`compas.geometry.FrameTarget`
            The Frame Target to calculate the inverse for.
        start_state : :class:`compas_fab.robots.RobotCellState`, optional
            The starting state to calculate the inverse kinematics for.
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
            - ``"timeout"``: (:obj:`int`, optional) Maximum allowed time for inverse kinematic calculation in seconds.
              Defaults to ``2``. This value supersedes the ``"attempts"`` argument used before ROS Noetic.
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`, optional)
              Defaults to ``None``.
            - ``"attempts"``: (:obj:`int`, optional) The maximum number of inverse kinematic attempts.
              Defaults to ``8``. This value is ignored on ROS Noetic and newer, use ``"timeout"`` instead.
            - ``"max_results"``: (:obj:`int`) Maximum number of results to return.
              Defaults to ``100``.

        Raises
        ------
        compas_fab.backends.exceptions.BackendError
            If no configuration can be found.

        Yields
        ------
        :obj:`tuple` of (:obj:`list` of :obj:`float`, :obj:`list` of :obj:`str`)
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.

        Examples
        --------

        """
        options = options or {}
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell

        # Group
        group = group or robot_cell.main_group_name

        # Set scene
        client.robot_cell.assert_cell_state_match(start_state)
        planner.set_robot_cell_state(start_state)

        # Start configuration
        start_configuration = start_state.robot_configuration
        # Merge with zero configuration ensure all joints are present
        start_configuration = robot_cell.zero_configuration(group).merged(start_configuration)

        # Target frame and Tool Coordinate Frame
        target = target.normalized_to_meters()
        target_frame = target.target_frame
        planner_frame = client.robot_cell.target_frames_to_pcf(start_state, target_frame, target.target_mode, group)

        # Compose the kwargs for the async function
        kwargs = {}
        kwargs["options"] = options
        kwargs["frame_WCF"] = planner_frame
        kwargs["group"] = group
        kwargs["start_configuration"] = start_configuration
        kwargs["errback_name"] = "errback"

        max_results = options.get("max_results", 100)

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", robot_cell.root_name)

        all_yielded_joint_positions = []
        # Yield up to max_results configurations
        for _ in range(max_results):
            # First iteration uses the provided start_configuration
            joint_positions, joint_names = await_callback(self.inverse_kinematics_async, **kwargs)

            # Check if the result is the same as any of the previous results
            result_is_unique = True
            for j in all_yielded_joint_positions:
                if TOL.is_allclose(joint_positions, j, rtol=TOL.APPROXIMATION):
                    result_is_unique = False

            # If result is unique, we yield it back to the user
            if result_is_unique:
                yield joint_positions, joint_names
                all_yielded_joint_positions.append(joint_positions)

            # Generate random start configuration for next iteration
            random_configuration = robot_cell.random_configuration(kwargs["group"])
            kwargs["start_configuration"] = random_configuration

    def inverse_kinematics_async(
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
        start_state = RobotState(joint_state, MultiDOFJointState(header=header), is_diff=True)

        # Filter needs to happen after all objects have been added
        start_state.filter_fields_for_distro(self.client.ros_distro)

        constraints = convert_constraints_to_rosmsg(options.get("constraints"), header)

        timeout_in_secs = options.get("timeout", 2)
        timeout_duration = {"secs": timeout_in_secs, "nsecs": 0}

        ik_request = PositionIKRequest(
            group_name=group,
            robot_state=start_state,
            constraints=constraints,
            pose_stamped=pose_stamped,
            avoid_collisions=options.get("avoid_collisions", True),
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

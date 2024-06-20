from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401
        from typing import Iterator  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401
        from typing import Dict  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import FrameTarget  # noqa: F401
        from compas_fab.robots import AttachedCollisionMesh  # noqa: F401


from compas.utilities import await_callback
from compas_fab.utilities import from_tcf_to_t0cf


from compas_fab.backends.interfaces import InverseKinematics
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import AttachedCollisionObject
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

from compas.tolerance import TOL

__all__ = [
    "MoveItInverseKinematics",
]


class MoveItInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""

    def __init__(self):
        # The following fields are used to store the last ik request for iterative calls
        self._last_ik_request = {"request_id": None, "solutions": None}
        # Initialize the super class
        print("MoveItInverseKinematics init")
        super(MoveItInverseKinematics, self).__init__()

    GET_POSITION_IK = ServiceDescription(
        "/compute_ik", "GetPositionIK", GetPositionIKRequest, GetPositionIKResponse, validate_response
    )

    # def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options=None):
    #     """
    #     Inverse kinematics calculation using MoveIt.

    #     """
    #     options = options or {}
    #     kwargs = {}
    #     kwargs["options"] = options
    #     kwargs["frame_WCF"] = frame_WCF
    #     kwargs["group"] = group or self.client.robot.main_group_name
    #     kwargs["start_configuration"] = start_configuration or self.client.robot.zero_configuration()
    #     kwargs["errback_name"] = "errback"

    #     # Use base_link or fallback to model's root link
    #     options["base_link"] = options.get("base_link", self.client.robot.model.root.name)

    #     return await_callback(self.inverse_kinematics_async, **kwargs)
    #     # return_full_configuration = options.get("return_full_configuration", False)
    #     # use_attached_tool_frame = options.get("use_attached_tool_frame", False)

    #     # # Pseudo-memoized sequential calls will re-use iterator if not exhausted
    #     # request_id = "{}-{}-{}-{}-{}".format(
    #     #     str(frame_WCF), str(start_configuration), str(group), str(return_full_configuration), str(options)
    #     # )

    #     # # Re-use last ik request generator if the request is the same
    #     # if self._last_ik_request["request_id"] == request_id and self._last_ik_request["solutions"] is not None:
    #     #     solution = next(self._last_ik_request["solutions"], None)
    #     #     if solution is not None:
    #     #         return solution

    #     # # Create new ik generator object
    #     # solutions = self.iter_inverse_kinematics(
    #     #     frame_WCF, start_configuration, group, return_full_configuration, use_attached_tool_frame, options
    #     # )
    #     # self._last_ik_request["request_id"] = request_id
    #     # self._last_ik_request["solutions"] = solutions

    #     # return next(solutions)

    def iter_inverse_kinematics(self, target, start_state=None, group=None, options=None):
        # type: (FrameTarget, Optional[RobotCellState], Optional[str], Optional[Dict]) -> Iterator[Tuple[List[float], List[str]]]
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
        """
        options = options or {}
        robot = self.client.robot

        # Group
        group = group or robot.main_group_name

        # Set scene
        # TODO: Implement start_state
        self.client.robot_cell.assert_cell_state_match(start_state)
        start_state = start_state or RobotCellState.from_robot_configuration(robot)

        # Start configuration
        start_configuration = start_state.robot_configuration
        # Merge with zero configuration ensure all joints are present
        start_configuration = robot.zero_configuration(group).merged(start_configuration)

        # Target frame and Tool Coordinate Frame
        target_frame = target.target_frame
        if robot.need_scaling:
            target_frame = target_frame.scaled(1.0 / robot.scale_factor)
            # Now target_frame is back in meter scale
        attached_tool = self.client.robot_cell.get_attached_tool(start_state, group)
        if attached_tool:
            target_frame = from_tcf_to_t0cf(target_frame, attached_tool.frame)
            # Attached tool frames does not need scaling because Tools are modelled in meter scale

        # Scale Attached Collision Meshes
        if robot.need_scaling:
            acms = options.get("attached_collision_meshes", [])  # type: List[AttachedCollisionMesh]
            for acm in acms:
                acm.collision_mesh.scale(1.0 / robot.scale_factor)

        # Compose the kwargs for the async function
        kwargs = {}
        kwargs["options"] = options
        kwargs["frame_WCF"] = target_frame
        kwargs["group"] = group
        kwargs["start_configuration"] = start_configuration or robot.zero_configuration()
        kwargs["errback_name"] = "errback"

        max_results = options.get("max_results", 100)

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", self.client.robot.model.root.name)

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
            random_configuration = self.client.robot.random_configuration(kwargs["group"])
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
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))

        if options.get("attached_collision_meshes"):
            for acm in options["attached_collision_meshes"]:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

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

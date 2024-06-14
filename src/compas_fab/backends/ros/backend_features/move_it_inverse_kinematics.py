from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

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

__all__ = [
    "MoveItInverseKinematics",
]


class MoveItInverseKinematics(InverseKinematics):
    """Callable to calculate the robot's inverse kinematics for a given frame."""

    GET_POSITION_IK = ServiceDescription(
        "/compute_ik", "GetPositionIK", GetPositionIKRequest, GetPositionIKResponse, validate_response
    )

    def inverse_kinematics(self, robot, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.

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
        :obj:`tuple` of :obj:`list`
            A tuple of 2 elements containing a list of joint positions and a list of matching joint names.
        """
        options = options or {}
        kwargs = {}
        kwargs["options"] = options
        kwargs["frame_WCF"] = frame_WCF
        kwargs["group"] = group
        kwargs["start_configuration"] = start_configuration
        kwargs["errback_name"] = "errback"

        max_results = options.get("max_results", 100)

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", robot.model.root.name)

        for _ in range(max_results):
            yield await_callback(self.inverse_kinematics_async, **kwargs)

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

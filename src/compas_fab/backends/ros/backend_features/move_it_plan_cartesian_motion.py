from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.backend_features.helpers import convert_trajectory
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import GetCartesianPathRequest
from compas_fab.backends.ros.messages import GetCartesianPathResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.service_description import ServiceDescription

from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints

__all__ = [
    "MoveItPlanCartesianMotion",
]


class MoveItPlanCartesianMotion(PlanCartesianMotion):
    """Callable to calculate a cartesian motion path (linear in tool space)."""

    GET_CARTESIAN_PATH = ServiceDescription(
        "/compute_cartesian_path",
        "GetCartesianPath",
        GetCartesianPathRequest,
        GetCartesianPathResponse,
        validate_response,
    )

    def plan_cartesian_motion(self, robot, waypoints, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the cartesian motion plan is being calculated.
        waypoints : :class:`compas_fab.robots.Waypoints`
            The waypoints for the robot to follow.
        start_configuration: :class:`compas_robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position. Defaults to
            the all-zero configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
            - ``"link"``: (:obj:`str`, optional) The name of the link to
              calculate the forward kinematics for. Defaults to the group's end
              effector link.
            - ``"max_step"``: (:obj:`float`, optional) The approximate distance between the
              calculated points. (Defined in the robot's units.) Defaults to ``0.01``.
            - ``"jump_threshold"``: (:obj:`float`, optional)
              The maximum allowed distance of joint positions between consecutive
              points. If the distance is found to be above this threshold, the
              path computation fails. It must be specified in relation to max_step.
              If this threshold is ``0``, 'jumps' might occur, resulting in an invalid
              cartesian path. Defaults to :math:`\\pi / 2`.
            - ``"avoid_collisions"``: (:obj:`bool`, optional)
              Whether or not to avoid collisions. Defaults to ``True``.
            - ``"path_constraints"``: (:obj:`list` of :class:`compas_fab.robots.Constraint`, optional)
              Optional constraints that can be imposed along the solution path.
              Note that path calculation won't work if the start_configuration
              violates these constraints. Defaults to ``None``.
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`)
              Defaults to ``None``.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        options = options or {}
        kwargs = {}
        kwargs["options"] = options
        kwargs["waypoints"] = waypoints
        kwargs["start_configuration"] = start_configuration
        kwargs["group"] = group

        kwargs["errback_name"] = "errback"

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", robot.model.root.name)
        options["joints"] = {j.name: j.type for j in robot.model.joints}

        options["link"] = options.get("link") or robot.get_end_effector_link_name(group)
        if options["link"] not in robot.get_link_names(group):
            raise ValueError("Link name {} does not exist in planning group".format(options["link"]))

        # This function wraps multiple implementations depending on the type of waypoints
        if isinstance(waypoints, FrameWaypoints):
            return await_callback(self.plan_cartesian_motion_with_frame_waypoints_async, **kwargs)
        elif isinstance(waypoints, PointAxisWaypoints):
            return self.plan_cartesian_motion_with_point_axis_waypoints_async(**kwargs)
        else:
            raise TypeError("Unsupported waypoints type {} for MoveIt planning backend.".format(type(waypoints)))

    def plan_cartesian_motion_with_frame_waypoints_async(
        self, callback, errback, waypoints, start_configuration=None, group=None, options=None
    ):
        """Asynchronous handler of MoveIt cartesian motion planner service.

        :class:`compas_fab.robots.FrameWaypoints` are converted to :class:`compas_fab.backends.ros.messages.Pose` that is native to ROS communication

        """

        joints = options["joints"]

        header = Header(frame_id=options["base_link"])

        # Convert compas_fab.robots.FrameWaypoints to list of Pose for ROS
        list_of_pose = [Pose.from_frame(frame) for frame in waypoints.target_frames]

        joint_state = JointState(
            header=header, name=start_configuration.joint_names, position=start_configuration.joint_values
        )
        start_state = RobotState(joint_state, MultiDOFJointState(header=header), is_diff=True)

        if options.get("attached_collision_meshes"):
            for acm in options["attached_collision_meshes"]:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        # Filter needs to happen after all objects have been added
        start_state.filter_fields_for_distro(self.client.ros_distro)

        path_constraints = convert_constraints_to_rosmsg(options.get("path_constraints"), header)

        request = dict(
            header=header,
            start_state=start_state,
            group_name=group,
            link_name=options["link"],
            waypoints=list_of_pose,
            max_step=float(options.get("max_step", 0.01)),
            jump_threshold=float(options.get("jump_threshold", 1.57)),
            avoid_collisions=bool(options.get("avoid_collisions", True)),
            path_constraints=path_constraints,
        )

        def response_handler(response):
            try:
                trajectory = convert_trajectory(
                    joints, response.solution, response.start_state, response.fraction, None, response
                )
                callback(trajectory)
            except Exception as e:
                errback(e)

        self.GET_CARTESIAN_PATH(self.client, request, response_handler, errback)

    def plan_cartesian_motion_with_point_axis_waypoints_async(
        self, callback, errback, waypoints, start_configuration=None, group=None, options=None
    ):
        """Asynchronous handler of MoveIt cartesian motion planner service.

        AFAIK MoveIt does not support planning for a relaxed axis under this
        """
        raise NotImplementedError("PointAxisWaypoints are not supported by MoveIt backend")

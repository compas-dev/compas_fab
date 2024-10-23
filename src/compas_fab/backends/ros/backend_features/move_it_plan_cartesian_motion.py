from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas.utilities import await_callback

from compas_fab.backends.interfaces import PlanCartesianMotion
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.backend_features.helpers import convert_trajectory
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import GetCartesianPathRequest
from compas_fab.backends.ros.messages import GetCartesianPathResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.service_description import ServiceDescription
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Callable  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas_fab.backends import MoveItPlanner  # noqa: F401
        from compas_fab.backends import RosClient  # noqa: F401
        from compas_fab.backends.ros.messages import PlanningScene  # noqa: F401
        from compas_fab.robots import JointTrajectory  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Waypoints  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401

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

    def plan_cartesian_motion(self, waypoints, start_state, group=None, options=None):
        # type: (Waypoints, RobotCellState, Optional[str], Optional[Dict]) -> JointTrajectory
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the cartesian motion plan is being calculated.
        waypoints : :class:`compas_fab.robots.Waypoints`
            The waypoints for the robot to follow.
        start_state : :class:`compas_fab.robots.RobotCellState`
            The starting state of the robot cell at the beginning of the motion.
            The attribute `robot_configuration`, must be provided.
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
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = planner.client  # type: RosClient # noqa: F841
        robot_cell = client.robot_cell  # type: RobotCell # noqa: F841

        options = options or {}
        kwargs = {}
        kwargs["options"] = options
        kwargs["waypoints"] = waypoints
        kwargs["start_state"] = start_state
        group = group or robot_cell.main_group_name
        kwargs["group"] = group

        kwargs["errback_name"] = "errback"

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", robot_cell.root_name)
        options["joints"] = {j.name: j.type for j in robot_cell.robot_model.joints}

        options["link"] = options.get("link") or robot_cell.get_end_effector_link_name(group)
        if options["link"] not in robot_cell.get_link_names(group):
            raise ValueError("Link name {} does not exist in planning group".format(options["link"]))

        # This function wraps multiple implementations depending on the type of waypoints
        if isinstance(waypoints, FrameWaypoints):
            return await_callback(self.plan_cartesian_motion_with_frame_waypoints_async, **kwargs)
        elif isinstance(waypoints, PointAxisWaypoints):
            return self.plan_cartesian_motion_with_point_axis_waypoints_async(**kwargs)
        else:
            raise TypeError("Unsupported waypoints type {} for MoveIt planning backend.".format(type(waypoints)))

    def plan_cartesian_motion_with_frame_waypoints_async(
        self, callback, errback, waypoints, start_state, group=None, options=None
    ):
        # type: (Callable, Callable, FrameWaypoints, RobotCellState, Optional[str], Optional[Dict]) -> None
        """Asynchronous handler of MoveIt cartesian motion planner service.

        :class:`compas_fab.robots.FrameWaypoints` are converted to :class:`compas_fab.backends.ros.messages.Pose` that is native to ROS communication

        """
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient

        joints = options["joints"]

        header = Header(frame_id=options["base_link"])

        # Convert compas_fab.robots.FrameWaypoints to list of Pose for ROS
        target_frames = waypoints.target_frames
        pcf_frames = client.robot_cell.target_frames_to_pcf(start_state, target_frames, waypoints.target_mode, group)
        list_of_pose = [Pose.from_frame(frame) for frame in pcf_frames]

        # We are calling the synchronous function here for simplicity.
        planner.set_robot_cell_state(start_state)
        planning_scene = planner.get_planning_scene()  # type: PlanningScene
        start_state = planning_scene.robot_state

        # start_configuration = start_state.robot_configuration
        # joint_state = JointState(
        #     header=header, name=start_configuration.joint_names, position=start_configuration.joint_values
        # )
        # start_state = RobotState(joint_state, MultiDOFJointState(header=header), is_diff=True)

        # if options.get("attached_collision_meshes"):
        #     for acm in options["attached_collision_meshes"]:
        #         aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
        #         start_state.attached_collision_objects.append(aco)

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
                print("Error Code:", response.error_code)
                trajectory = convert_trajectory(
                    joints, response.solution, response.start_state, response.fraction, None, response
                )
                callback(trajectory)
            except Exception as e:
                errback(e)

        self.GET_CARTESIAN_PATH(client, request, response_handler, errback)

    def plan_cartesian_motion_with_point_axis_waypoints_async(
        self, callback, errback, waypoints, start_state=None, group=None, options=None
    ):
        """Asynchronous handler of MoveIt cartesian motion planner service.

        AFAIK MoveIt does not support planning for a relaxed axis under this
        """
        raise NotImplementedError("PointAxisWaypoints are not supported by MoveIt backend")

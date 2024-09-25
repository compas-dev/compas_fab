from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import PlanMotion
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.backend_features.helpers import convert_target_to_goal_constraints
from compas_fab.backends.ros.backend_features.helpers import convert_trajectory
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MotionPlanRequest
from compas_fab.backends.ros.messages import MotionPlanResponse
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages import TrajectoryConstraints
from compas_fab.backends.ros.service_description import ServiceDescription

import compas

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import MoveItPlanner  # noqa: F401
        from compas_fab.backends import RosClient  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401

__all__ = ["MoveItPlanMotion"]


class MoveItPlanMotion(PlanMotion):
    """Callable to find a path in joint space for the robot to move from its `start_configuration` to the `target`."""

    GET_MOTION_PLAN = ServiceDescription(
        "/plan_kinematic_path", "GetMotionPlan", MotionPlanRequest, MotionPlanResponse, validate_response
    )

    def plan_motion(self, robot, target, start_configuration=None, group=None, options=None):
        """Calculates a motion path.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the motion plan is being calculated.
        target: list of :class:`compas_fab.robots.Target`
            The target for the robot to reach.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position. Defaults to
            the all-zero configuration.
        group: str, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
            - ``"path_constraints"``: (:obj:`list` of :class:`compas_fab.robots.Constraint`, optional)
              Optional constraints that can be imposed along the solution path.
              Note that path calculation won't work if the start_configuration
              violates these constraints. Defaults to ``None``.
            - ``"planner_id"``: (:obj:`str`)
              The name of the algorithm used for path planning.
              Defaults to ``'RRTConnect'``.
            - ``"num_planning_attempts"``: (:obj:`int`, optional)
              Normally, if one motion plan is needed, one motion plan is computed.
              However, for algorithms that use randomization in their execution
              (like 'RRT'), it is likely that different planner executions will
              produce different solutions. Setting this parameter to a value above
              ``1`` will run many additional motion plans, and will report the
              shortest solution as the final result. Defaults to ``1``.
            - ``'allowed_planning_time'``: (:obj:`float`)
              The number of seconds allowed to perform the planning. Defaults to ``2``.
            - ``"max_velocity_scaling_factor"``: (:obj:`float`)
              Defaults to ``1``.
            - ``"max_acceleration_scaling_factor"``: (:obj:`float`)
              Defaults to ``1``.
            - ``"attached_collision_meshes"``: (:obj:`list` of :class:`compas_fab.robots.AttachedCollisionMesh`)
              Defaults to ``None``.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        options = options or {}
        kwargs = {}
        kwargs["target"] = target
        kwargs["start_configuration"] = start_configuration
        kwargs["group"] = group
        kwargs["options"] = options
        kwargs["errback_name"] = "errback"

        # Use base_link or fallback to model's root link
        options["base_link"] = options.get("base_link", robot.model.root.name)
        options["joints"] = {j.name: j.type for j in robot.model.joints}
        options["ee_link_name"] = robot.get_end_effector_link_name(group)

        return await_callback(self.plan_motion_async, **kwargs)

    def plan_motion_async(self, callback, errback, target, start_configuration=None, group=None, options=None):
        """Asynchronous handler of MoveIt motion planner service."""
        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: if list of frames (goals) => receive multiple solutions?

        # Housekeeping for intellisense
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot = client.robot  # type: Robot # noqa: F841

        joints = options["joints"]
        header = Header(frame_id=options["base_link"])
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

        # Convert targets to constraints, and to ROS message
        ee_link_name = options["ee_link_name"]
        attached_tool_id = client.robot_cell_state.get_attached_tool_id(group)
        if attached_tool_id:
            tool_coordinate_frame = client.robot_cell_state.tool_states[attached_tool_id].attachment_frame
        else:
            tool_coordinate_frame = None
        goal_constraints = convert_target_to_goal_constraints(target, ee_link_name, tool_coordinate_frame)
        goal_constraints = [convert_constraints_to_rosmsg(goal_constraints, header)]
        path_constraints = convert_constraints_to_rosmsg(options.get("path_constraints"), header)
        trajectory_constraints = options.get("trajectory_constraints")

        if trajectory_constraints is not None:
            trajectory_constraints = TrajectoryConstraints(
                constraints=convert_constraints_to_rosmsg(options["trajectory_constraints"], header)
            )

        request = dict(
            start_state=start_state,
            goal_constraints=goal_constraints,
            path_constraints=path_constraints,
            trajectory_constraints=trajectory_constraints,
            planner_id=options.get("planner_id", "RRTConnect"),
            group_name=group,
            num_planning_attempts=options.get("num_planning_attempts", 1),
            allowed_planning_time=options.get("allowed_planning_time", 2.0),
            max_velocity_scaling_factor=options.get("max_velocity_scaling_factor", 1.0),
            max_acceleration_scaling_factor=options.get("max_acceleration_scaling_factor", 1.0),
        )
        # workspace_parameters=options.get('workspace_parameters')

        def response_handler(response):
            try:
                trajectory = convert_trajectory(
                    joints, response.trajectory, response.trajectory_start, 1.0, response.planning_time, response
                )
                callback(trajectory)
            except Exception as e:
                errback(e)

        self.GET_MOTION_PLAN(self.client, request, response_handler, errback)

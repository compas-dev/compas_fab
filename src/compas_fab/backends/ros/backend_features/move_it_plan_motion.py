from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas.utilities import await_callback

from compas_fab.backends.interfaces import PlanMotion
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
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

from compas_fab.robots import ConstraintSetTarget
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget

from compas_fab.robots.constraints import JointConstraint
from compas_fab.robots.constraints import PositionConstraint
from compas_fab.robots.constraints import OrientationConstraint


if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Optional  # noqa: F401

        from compas_fab.backends import MoveItPlanner  # noqa: F401
        from compas_fab.backends import RosClient  # noqa: F401
        from compas_fab.robots import JointTrajectory  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Target  # noqa: F401

__all__ = ["MoveItPlanMotion"]


class MoveItPlanMotion(PlanMotion):
    """Callable to find a path in joint space for the robot to move from its `start_configuration` to the `target`."""

    DEFAULT_TOLERANCE_ORIENTATION = 0.01
    DEFAULT_TOLERANCE_POSITION = 0.001
    DEFAULT_TOLERANCE_JOINT = 0.01

    GET_MOTION_PLAN = ServiceDescription(
        "/plan_kinematic_path", "GetMotionPlan", MotionPlanRequest, MotionPlanResponse, validate_response
    )

    def plan_motion(self, target, start_state, group=None, options=None):
        # type: (Target, RobotCellState, Optional[str], Optional[dict]) -> JointTrajectory
        """Calculates a motion path.

        The PyBullet Planner supports ConstraintSetTarget, ConfigurationTarget and FrameTarget.

        Parameters
        ----------
        target : :class:`compas_fab.robots.Target`
            The goal for the robot to achieve.
        start_state : :class:`compas_fab.robots.RobotCellState`
            The starting state of the robot cell at the beginning of the motion.
            The attribute `robot_configuration`, must be provided.
        group: str, optional
            The name of the group to plan for. Defaults to the robot's main
            planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
              Defaults to the robot's root link.
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

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell  # type: RobotCell

        # Default Options
        options = options or {}
        options["base_link"] = options.get("base_link", robot_cell.root_name)
        options["path_constraints"] = options.get("path_constraints", None)
        options["planner_id"] = options.get("planner_id", "RRTConnect")
        options["num_planning_attempts"] = options.get("num_planning_attempts", 1)
        options["allowed_planning_time"] = options.get("allowed_planning_time", 2.0)
        options["max_velocity_scaling_factor"] = options.get("max_velocity_scaling_factor", 1.0)
        options["max_acceleration_scaling_factor"] = options.get("max_acceleration_scaling_factor", 1.0)

        # Setting the entire robot cell state, including the robot configuration
        planner.set_robot_cell_state(start_state)

        # Parameters for `plan_motion_async``
        kwargs = {}
        kwargs["target"] = target
        kwargs["start_state"] = start_state
        kwargs["group"] = group or robot_cell.main_group_name
        kwargs["options"] = options
        kwargs["errback_name"] = "errback"

        return await_callback(self._plan_motion_async, **kwargs)

    def _plan_motion_async(self, callback, errback, target, start_state, group, options):
        # type: (callable, callable, Target, RobotCellState, str, dict) -> JointTrajectory
        """Asynchronous handler of MoveIt motion planner service."""
        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: if list of frames (goals) => receive multiple solutions?

        # Housekeeping for intellisense
        planner = self  # type: MoveItPlanner
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell  # type: RobotCell # noqa: F841

        ee_link_name = robot_cell.get_end_effector_link_name(group)

        # Convert target to Constraints
        goal_constraints = []

        if isinstance(target, ConstraintSetTarget):
            goal_constraints = target.constraint_set

        elif isinstance(target, ConfigurationTarget):
            configuration = target.target_configuration
            tolerance_above = target.tolerance_above or self.DEFAULT_TOLERANCE_JOINT
            tolerance_below = target.tolerance_below or self.DEFAULT_TOLERANCE_JOINT
            goal_constraints = JointConstraint.joint_constraints_from_configuration(
                configuration, tolerance_above, tolerance_below
            )

        elif isinstance(target, FrameTarget):
            target_pcf = client.robot_cell.target_frames_to_pcf(
                start_state, target.target_frame, target.target_mode, group
            )
            tolerance_position = target.tolerance_position or self.DEFAULT_TOLERANCE_POSITION
            tolerance_orientation = target.tolerance_orientation or self.DEFAULT_TOLERANCE_ORIENTATION
            pc = PositionConstraint.from_frame(target_pcf, tolerance_position, ee_link_name)
            oc = OrientationConstraint.from_frame(target_pcf, [tolerance_orientation] * 3, ee_link_name)
            goal_constraints = [pc, oc]

        elif isinstance(target, PointAxisTarget):
            raise NotImplementedError("Target type {} not supported by ROS planning backend.".format(type(target)))
        else:
            raise NotImplementedError("Target type {} not supported by ROS planning backend.".format(type(target)))

        # ===================================================================================
        # Formatting input for ROS MoveIt
        # ===================================================================================
        joints = {j.name: j.type for j in robot_cell.robot_model.joints}
        header = Header(frame_id=options["base_link"])

        # Convert constraints to ROS message
        goal_constraints = [convert_constraints_to_rosmsg(goal_constraints, header)]
        path_constraints = convert_constraints_to_rosmsg(options.get("path_constraints"), header)

        # Preprocess trajectory constraints
        trajectory_constraints = options.get("trajectory_constraints")
        if trajectory_constraints is not None:
            trajectory_constraints = TrajectoryConstraints(
                constraints=convert_constraints_to_rosmsg(options["trajectory_constraints"], header)
            )

        # Convert start state to ROS JointState and RobotState
        start_configuration = start_state.robot_configuration
        ros_joint_state = JointState(
            header=header, name=start_configuration.joint_names, position=start_configuration.joint_values
        )
        ros_start_state = RobotState(ros_joint_state, MultiDOFJointState(header=header), is_diff=True)
        ros_start_state.filter_fields_for_distro(self.client.ros_distro)

        request = dict(
            start_state=ros_start_state,
            goal_constraints=goal_constraints,
            path_constraints=path_constraints,
            trajectory_constraints=trajectory_constraints,
            planner_id=options["planner_id"],
            group_name=group,
            num_planning_attempts=options["num_planning_attempts"],
            allowed_planning_time=options["allowed_planning_time"],
            max_velocity_scaling_factor=options["max_velocity_scaling_factor"],
            max_acceleration_scaling_factor=options["max_acceleration_scaling_factor"],
        )

        def response_handler(response):
            try:
                trajectory = convert_trajectory(
                    joints, response.trajectory, response.trajectory_start, 1.0, response.planning_time, response
                )
                callback(trajectory)
            except Exception as e:
                errback(e)

        self.GET_MOTION_PLAN(self.client, request, response_handler, errback)

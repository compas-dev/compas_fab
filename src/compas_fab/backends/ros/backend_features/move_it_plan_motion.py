from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import PlanMotion
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.backend_features.helpers import convert_trajectory_points
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MotionPlanResponse
from compas_fab.backends.ros.messages import MotionPlanRequest
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages import TrajectoryConstraints
from compas_fab.backends.ros.service_description import ServiceDescription
from compas_fab.robots import JointTrajectory
from compas_fab.robots import Configuration

__all__ = [
    'MoveItPlanMotion'
]


class MoveItPlanMotion(PlanMotion):
    """Callable to find a path plan to move the selected robot from its current position within the `goal_constraints`."""
    GET_MOTION_PLAN = ServiceDescription('/plan_kinematic_path',
                                         'GetMotionPlan',
                                         MotionPlanRequest,
                                         MotionPlanResponse,
                                         validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def plan_motion(self, robot, goal_constraints, start_configuration=None, group=None, options=None):
        """Calculates a motion path.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the motion plan is being calculated.
        goal_constraints: list of :class:`compas_fab.robots.Constraint`
            The goal to be achieved, defined in a set of constraints.
            Constraints can be very specific, for example defining value domains
            for each joint, such that the goal configuration is included,
            or defining a volume in space, to which a specific robot link (e.g.
            the end-effector) is required to move to.
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
              Defaults to ``'RRTConnectkConfigDefault'``.
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
        kwargs['goal_constraints'] = goal_constraints
        kwargs['start_configuration'] = start_configuration
        kwargs['group'] = group
        kwargs['options'] = options
        kwargs['errback_name'] = 'errback'

        # Use base_link or fallback to model's root link
        options['base_link'] = options.get('base_link', robot.model.root.name)
        options['joint_names'] = robot.get_configurable_joint_names()
        options['joint_types'] = robot.get_configurable_joint_types()

        return await_callback(self.plan_motion_async, **kwargs)

    def plan_motion_async(self, callback, errback,
                          goal_constraints, start_configuration=None, group=None, options=None):
        """Asynchronous handler of MoveIt motion planner service."""

        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: if list of frames (goals) => receive multiple solutions?
        joint_names = options['joint_names']
        joint_types = options['joint_types']
        joint_type_by_name = dict(zip(joint_names, joint_types))  # !!! should this go somewhere else? does it already exist somewhere else?

        header = Header(frame_id=options['base_link'])
        joint_state = JointState(
            header=header,
            name=start_configuration.joint_names,
            position=start_configuration.values)
        start_state = RobotState(
            joint_state, MultiDOFJointState(header=header))
        if options.get('attached_collision_meshes'):
            for acm in options['attached_collision_meshes']:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        # convert constraints
        goal_constraints = [convert_constraints_to_rosmsg(goal_constraints, header)]
        path_constraints = convert_constraints_to_rosmsg(options.get('path_constraints'), header)
        trajectory_constraints = options.get('trajectory_constraints')

        if trajectory_constraints is not None:
            trajectory_constraints = TrajectoryConstraints(constraints=convert_constraints_to_rosmsg(options['trajectory_constraints'], header))

        request = dict(start_state=start_state,
                       goal_constraints=goal_constraints,
                       path_constraints=path_constraints,
                       trajectory_constraints=trajectory_constraints,
                       planner_id=options.get('planner_id', 'RRTConnectkConfigDefault'),
                       group_name=group,
                       num_planning_attempts=options.get('num_planning_attempts', 1),
                       allowed_planning_time=options.get('allowed_planning_time', 2.),
                       max_velocity_scaling_factor=options.get('max_velocity_scaling_factor', 1.),
                       max_acceleration_scaling_factor=options.get('max_acceleration_scaling_factor', 1.))
        # workspace_parameters=options.get('workspace_parameters')

        def convert_to_trajectory(response):
            trajectory = JointTrajectory()
            trajectory.source_message = response
            trajectory.fraction = 1.
            trajectory.joint_names = response.trajectory.joint_trajectory.joint_names
            trajectory.planning_time = response.planning_time

            joint_types = [joint_type_by_name[name] for name in trajectory.joint_names]
            trajectory.points = convert_trajectory_points(
                response.trajectory.joint_trajectory.points, joint_types)

            start_state = response.trajectory_start.joint_state
            start_state_types = [joint_type_by_name[name] for name in start_state.name]
            trajectory.start_configuration = Configuration(start_state.position, start_state_types, start_state.name)

            callback(trajectory)

        self.GET_MOTION_PLAN(self.ros_client, request, convert_to_trajectory, errback)

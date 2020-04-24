from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

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
from compas_fab.backends.ros.planner_backend import ServiceDescription
from compas_fab.robots import JointTrajectory
from compas_fab.robots import Configuration


class MoveItPlanMotion(object):
    GET_MOTION_PLAN = ServiceDescription('/plan_kinematic_path',
                                         'GetMotionPlan',
                                         MotionPlanRequest,
                                         MotionPlanResponse,
                                         validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def __call__(self, robot, goal_constraints, start_configuration, group,
                 path_constraints=None, trajectory_constraints=None,
                 planner_id='', num_planning_attempts=8,
                 allowed_planning_time=2.,
                 max_velocity_scaling_factor=1.,
                 max_acceleration_scaling_factor=1.,
                 attached_collision_meshes=None,
                 workspace_parameters=None):
        return self.plan_motion(robot, goal_constraints, start_configuration, group,
                                path_constraints, trajectory_constraints,
                                planner_id, num_planning_attempts,
                                allowed_planning_time,
                                max_velocity_scaling_factor,
                                max_acceleration_scaling_factor,
                                attached_collision_meshes,
                                workspace_parameters)

    def plan_motion(self, robot, goal_constraints, start_configuration, group,
                    path_constraints=None, trajectory_constraints=None,
                    planner_id='', num_planning_attempts=8,
                    allowed_planning_time=2.,
                    max_velocity_scaling_factor=1.,
                    max_acceleration_scaling_factor=1.,
                    attached_collision_meshes=None,
                    workspace_parameters=None):
        kwargs = {}
        kwargs['robot'] = robot
        kwargs['goal_constraints'] = goal_constraints
        kwargs['start_configuration'] = start_configuration
        kwargs['group'] = group
        kwargs['path_constraints'] = path_constraints
        kwargs['trajectory_constraints'] = trajectory_constraints
        kwargs['planner_id'] = planner_id
        kwargs['num_planning_attempts'] = num_planning_attempts
        kwargs['allowed_planning_time'] = allowed_planning_time
        kwargs['max_velocity_scaling_factor'] = max_velocity_scaling_factor
        kwargs['max_acceleration_scaling_factor'] = max_acceleration_scaling_factor
        kwargs['attached_collision_meshes'] = attached_collision_meshes
        kwargs['workspace_parameters'] = workspace_parameters

        kwargs['errback_name'] = 'errback'

        return await_callback(self.plan_motion_async, **kwargs)

    def plan_motion_async(self, callback, errback,
                          robot, goal_constraints, start_configuration, group,
                          path_constraints=None, trajectory_constraints=None,
                          planner_id='', num_planning_attempts=8,
                          allowed_planning_time=2.,
                          max_velocity_scaling_factor=1.,
                          max_acceleration_scaling_factor=1.,
                          attached_collision_meshes=None,
                          workspace_parameters=None):
        """Asynchronous handler of MoveIt motion planner service."""

        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: if list of frames (goals) => receive multiple solutions?
        base_link = robot.model.root.name  # use world coords

        header = Header(frame_id=base_link)
        joint_state = JointState(
            header=header,
            name=start_configuration.joint_names,
            position=start_configuration.values)
        start_state = RobotState(
            joint_state, MultiDOFJointState(header=header))
        if attached_collision_meshes:
            for acm in attached_collision_meshes:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        # convert constraints
        goal_constraints = [convert_constraints_to_rosmsg(goal_constraints, header)]
        path_constraints = convert_constraints_to_rosmsg(path_constraints, header)

        if trajectory_constraints is not None:
            trajectory_constraints = TrajectoryConstraints(constraints=convert_constraints_to_rosmsg(path_constraints, header))

        request = dict(start_state=start_state,
                       goal_constraints=goal_constraints,
                       path_constraints=path_constraints,
                       trajectory_constraints=trajectory_constraints,
                       planner_id=planner_id,
                       group_name=group,
                       num_planning_attempts=num_planning_attempts,
                       allowed_planning_time=allowed_planning_time,
                       max_velocity_scaling_factor=max_velocity_scaling_factor,
                       max_acceleration_scaling_factor=max_velocity_scaling_factor)
        # workspace_parameters=workspace_parameters

        def convert_to_trajectory(response):
            trajectory = JointTrajectory()
            trajectory.source_message = response
            trajectory.fraction = 1.
            trajectory.joint_names = response.trajectory.joint_trajectory.joint_names
            trajectory.planning_time = response.planning_time

            joint_types = robot.get_joint_types_by_names(trajectory.joint_names)
            trajectory.points = convert_trajectory_points(
                response.trajectory.joint_trajectory.points, joint_types)

            start_state = response.trajectory_start.joint_state
            start_state_types = robot.get_joint_types_by_names(start_state.name)
            trajectory.start_configuration = Configuration(start_state.position, start_state_types, start_state.name)

            callback(trajectory)

        self.GET_MOTION_PLAN(self.ros_client, request, convert_to_trajectory, errback)

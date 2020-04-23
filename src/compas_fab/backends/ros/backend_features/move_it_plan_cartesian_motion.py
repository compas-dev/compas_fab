from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.ros.messages import Header, Pose, JointState, RobotState, MultiDOFJointState, AttachedCollisionObject, GetCartesianPathRequest, \
    GetCartesianPathResponse
from compas_fab.backends.ros.backend_features.helpers import _convert_constraints_to_rosmsg, convert_trajectory_points, validate_response
from compas_fab.backends.ros.planner_backend import ServiceDescription
from compas_fab.robots import Configuration, JointTrajectory


class MoveItPlanCartesianMotion(object):
    GET_CARTESIAN_PATH = ServiceDescription('/compute_cartesian_path',
                                            'GetCartesianPath',
                                            GetCartesianPathRequest,
                                            GetCartesianPathResponse,
                                            validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def __call__(self, robot, frames, start_configuration,
                 group, max_step, jump_threshold,
                 avoid_collisions, path_constraints,
                 attached_collision_meshes):
        return self.plan_cartesian_motion( robot, frames, start_configuration,
                                           group, max_step, jump_threshold,
                                           avoid_collisions, path_constraints,
                                           attached_collision_meshes)

    def plan_cartesian_motion(self,
                              robot, frames, start_configuration,
                              group, max_step, jump_threshold,
                              avoid_collisions, path_constraints,
                              attached_collision_meshes):
        kwargs = {}
        kwargs['robot'] = robot
        kwargs['frames'] = frames
        kwargs['start_configuration'] = start_configuration
        kwargs['group'] = group
        kwargs['max_step'] = max_step
        kwargs['jump_threshold'] = jump_threshold
        kwargs['avoid_collisions'] = avoid_collisions
        kwargs['path_constraints'] = path_constraints
        kwargs['attached_collision_meshes'] = attached_collision_meshes

        kwargs['errback_name'] = 'errback'

        return await_callback(self.plan_cartesian_motion_async, **kwargs)

    def plan_cartesian_motion_async(self, callback, errback,
                                    robot, frames, start_configuration,
                                    group, max_step, jump_threshold,
                                    avoid_collisions, path_constraints,
                                    attached_collision_meshes):
        """Asynchronous handler of MoveIt cartesian motion planner service."""
        base_link = robot.model.root.name  # use world coords
        ee_link = robot.get_end_effector_link_name(group)

        header = Header(frame_id=base_link)
        waypoints = [Pose.from_frame(frame) for frame in frames]
        joint_state = JointState(header=header,
                                 name=start_configuration.joint_names,
                                 position=start_configuration.values)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))

        if attached_collision_meshes:
            for acm in attached_collision_meshes:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        path_constraints = _convert_constraints_to_rosmsg(path_constraints, header)

        request = dict(header=header,
                       start_state=start_state,
                       group_name=group,
                       link_name=ee_link,
                       waypoints=waypoints,
                       max_step=float(max_step),
                       jump_threshold=float(jump_threshold),
                       avoid_collisions=bool(avoid_collisions),
                       path_constraints=path_constraints)

        def convert_to_trajectory(response):
            try:
                trajectory = JointTrajectory()
                trajectory.source_message = response
                trajectory.fraction = response.fraction
                trajectory.joint_names = response.solution.joint_trajectory.joint_names

                joint_types = robot.get_joint_types_by_names(trajectory.joint_names)
                trajectory.points = convert_trajectory_points(
                    response.solution.joint_trajectory.points, joint_types)

                start_state = response.start_state.joint_state
                start_state_types = robot.get_joint_types_by_names(start_state.name)
                trajectory.start_configuration = Configuration(start_state.position, start_state_types, start_state.name)

                callback(trajectory)

            except Exception as e:
                errback(e)

        self.GET_CARTESIAN_PATH(self.ros_client, request, convert_to_trajectory, errback)

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.backend_feature_interfaces import PlanCartesianMotion
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.backend_features.helpers import convert_trajectory_points
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import GetCartesianPathRequest
from compas_fab.backends.ros.messages import GetCartesianPathResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.planner_backend import ServiceDescription
from compas_fab.robots import Configuration
from compas_fab.robots import JointTrajectory


class MoveItPlanCartesianMotion(PlanCartesianMotion):
    GET_CARTESIAN_PATH = ServiceDescription('/compute_cartesian_path',
                                            'GetCartesianPath',
                                            GetCartesianPathRequest,
                                            GetCartesianPathResponse,
                                            validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def plan_cartesian_motion(self, frames_WCF, start_configuration=None, group=None, options={}):
        base_link = options.get('base_link')
        ee_link = options.get('ee_link')
        joint_names = options.get('joint_names')
        joint_types = options.get('joint_types')
        max_step = options.get('max_step')
        jump_threshold = options.get('jump_threshold')
        avoid_collisions = options.get('avoid_collisions')
        path_constraints = options.get('path_constraints')
        attached_collision_meshes = options.get('attached_collision_meshes')
        return self.plan_cartesian_motion_deprecated(base_link, ee_link, joint_names, joint_types,
                                                     frames_WCF, start_configuration,
                                                     group, max_step, jump_threshold,
                                                     avoid_collisions, path_constraints,
                                                     attached_collision_meshes)

    def plan_cartesian_motion_deprecated(self,
                                         base_link, ee_link, joint_names, joint_types,
                                         frames, start_configuration,
                                         group, max_step, jump_threshold,
                                         avoid_collisions, path_constraints,
                                         attached_collision_meshes):
        kwargs = {}
        kwargs['base_link'] = base_link
        kwargs['ee_link'] = ee_link
        kwargs['joint_names'] = joint_names
        kwargs['joint_types'] = joint_types
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
                                    base_link, ee_link,  joint_names, joint_types,
                                    frames, start_configuration,
                                    group, max_step, jump_threshold,
                                    avoid_collisions, path_constraints,
                                    attached_collision_meshes):
        """Asynchronous handler of MoveIt cartesian motion planner service."""
        joint_type_by_name = dict(zip(joint_names, joint_types))  # should this go somewhere else? does it already exist somewhere else?

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

        path_constraints = convert_constraints_to_rosmsg(path_constraints, header)

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

                joint_types = [joint_type_by_name[name] for name in trajectory.joint_names]
                trajectory.points = convert_trajectory_points(
                    response.solution.joint_trajectory.points, joint_types)

                start_state = response.start_state.joint_state
                start_state_types = [joint_type_by_name[name] for name in start_state.name]
                trajectory.start_configuration = Configuration(start_state.position, start_state_types, start_state.name)

                callback(trajectory)

            except Exception as e:
                errback(e)

        self.GET_CARTESIAN_PATH(self.ros_client, request, convert_to_trajectory, errback)

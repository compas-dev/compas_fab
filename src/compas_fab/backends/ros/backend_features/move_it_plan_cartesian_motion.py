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
from compas_fab.backends.ros.service_description import ServiceDescription
from compas_fab.robots import Configuration
from compas_fab.robots import JointTrajectory

__all__ = [
    'MoveItPlanCartesianMotion',
]


class MoveItPlanCartesianMotion(PlanCartesianMotion):
    """Callable to calculate a cartesian motion path (linear in tool space)."""
    GET_CARTESIAN_PATH = ServiceDescription('/compute_cartesian_path',
                                            'GetCartesianPath',
                                            GetCartesianPathRequest,
                                            GetCartesianPathResponse,
                                            validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def plan_cartesian_motion(self, frames_WCF, start_configuration=None, group=None, options={}):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        frames_WCF: list of :class:`compas.geometry.Frame`
            The frames through which the path is defined.
        start_configuration: :class:`Configuration`, optional
            The robot's full configuration, i.e. values for all configurable
            joints of the entire robot, at the starting position. Defaults to
            the all-zero configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - base_link (:obj:`str`) :: Name of the base link.
            - ee_link (:obj:`str`) :: Name of the end effector link.
            - joint_names (:obj:`list` of :obj:`str`) :: List containing joint names.
            - joint_types (:obj:`list` of :obj:`str`) :: List containing joint types.
            - max_step :: float, optional
                The approximate distance between the calculated points. (Defined in
                the robot's units.) Defaults to `0.01`.
            - jump_threshold :: float, optional
                The maximum allowed distance of joint positions between consecutive
                points. If the distance is found to be above this threshold, the
                path computation fails. It must be specified in relation to max_step.
                If this threshold is 0, 'jumps' might occur, resulting in an invalid
                cartesian path. Defaults to pi/2.
            - avoid_collisions :: bool, optional
                Whether or not to avoid collisions. Defaults to `True`.
            - path_constraints :: list of :class:`compas_fab.robots.Constraint`, optional
                Optional constraints that can be imposed along the solution path.
                Note that path calculation won't work if the start_configuration
                violates these constraints. Defaults to `None`.
            - attached_collision_meshes :: list of :class:`compas_fab.robots.AttachedCollisionMesh`
                Defaults to `None`.

        Returns
        -------
        :class:`compas_fab.robots.JointTrajectory`
            The calculated trajectory.
        """
        kwargs = {}
        kwargs['base_link'] = options['base_link']
        kwargs['ee_link'] = options['ee_link']
        kwargs['joint_names'] = options['joint_names']
        kwargs['joint_types'] = options['joint_types']
        kwargs['frames'] = frames_WCF
        kwargs['start_configuration'] = start_configuration
        kwargs['group'] = group
        kwargs['max_step'] = options.get('max_step', 0.01)
        kwargs['jump_threshold'] = options.get('jump_threshold', 1.57)
        kwargs['avoid_collisions'] = options.get('avoid_collisions', True)
        kwargs['path_constraints'] = options.get('path_constraints')
        kwargs['attached_collision_meshes'] = options.get('attached_collision_meshes')

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

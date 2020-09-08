from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import PlanCartesianMotion
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

    def plan_cartesian_motion(self, robot, frames_WCF, start_configuration=None, group=None, options=None):
        """Calculates a cartesian motion path (linear in tool space).

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which the cartesian motion plan is being calculated.
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
        kwargs['options'] = options
        kwargs['frames_WCF'] = frames_WCF
        kwargs['start_configuration'] = start_configuration
        kwargs['group'] = group

        kwargs['errback_name'] = 'errback'

        # Use base_link or fallback to model's root link
        options['base_link'] = options.get('base_link', robot.model.root.name)
        options['joint_names'] = robot.get_configurable_joint_names()
        options['joint_types'] = robot.get_configurable_joint_types()
        options['link'] = options.get('link') or robot.get_end_effector_link_name(group)
        if options['link'] not in robot.get_link_names(group):
            raise ValueError('Link name {} does not exist in planning group'.format(options['link']))

        return await_callback(self.plan_cartesian_motion_async, **kwargs)

    def plan_cartesian_motion_async(self, callback, errback,
                                    frames_WCF, start_configuration=None, group=None, options=None):
        """Asynchronous handler of MoveIt cartesian motion planner service."""
        joint_names = options['joint_names']
        joint_types = options['joint_types']
        joint_type_by_name = dict(zip(joint_names, joint_types))

        header = Header(frame_id=options['base_link'])
        waypoints = [Pose.from_frame(frame) for frame in frames_WCF]
        joint_state = JointState(header=header,
                                 name=start_configuration.joint_names,
                                 position=start_configuration.values)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))

        if options.get('attached_collision_meshes'):
            for acm in options['attached_collision_meshes']:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        path_constraints = convert_constraints_to_rosmsg(options.get('path_constraints'), header)

        request = dict(header=header,
                       start_state=start_state,
                       group_name=group,
                       link_name=options['link'],
                       waypoints=waypoints,
                       max_step=float(options.get('max_step', 0.01)),
                       jump_threshold=float(options.get('jump_threshold', 1.57)),
                       avoid_collisions=bool(options.get('avoid_collisions', True)),
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

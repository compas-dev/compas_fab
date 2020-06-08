from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.backend_feature_interfaces import InverseKinematics
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.backend_features.helpers import convert_constraints_to_rosmsg
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import GetPositionIKRequest
from compas_fab.backends.ros.messages import GetPositionIKResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.messages import PoseStamped
from compas_fab.backends.ros.messages import PositionIKRequest
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    'MoveItInverseKinematics',
]


class MoveItInverseKinematics(InverseKinematics):
    GET_POSITION_IK = ServiceDescription('/compute_ik',
                                         'GetPositionIK',
                                         GetPositionIKRequest,
                                         GetPositionIKResponse,
                                         validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def inverse_kinematics(self, frame_WCF, start_configuration=None, group=None, options=None):
        """Calculate the robot's inverse kinematic for a given frame.

        Parameters
        ----------
        frame_WCF: :class:`compas.geometry.Frame`
            The frame to calculate the inverse for.
        start_configuration: :class:`compas_fab.robots.Configuration`, optional
            If passed, the inverse will be calculated such that the calculated
            joint positions differ the least from the start_configuration.
            Defaults to the init configuration.
        group: str, optional
            The planning group used for calculation. Defaults to the robot's
            main planning group.
        options: dict, optional
            Dictionary containing the following key-value pairs:

            - base_link (:obj:`str`) :: Name of the base link.
            - avoid_collisions :: bool, optional
                Whether or not to avoid collisions. Defaults to `True`.
            - constraints :: list of :class:`compas_fab.robots.Constraint`, optional
                A set of constraints that the request must obey. Defaults to `None`.
            - attempts :: int, optional
                The maximum number of inverse kinematic attempts. Defaults to `8`.
            - attached_collision_meshes :: list of :class:`compas_fab.robots.AttachedCollisionMesh`
                Defaults to `None`.
            - return_full_configuration :: bool
                If ``True``, returns a full configuration with all joint values
                specified, including passive ones if available.

        Raises
        ------
        compas_fab.backends.exceptions.BackendError
            If no configuration can be found.

        Returns
        -------
        :class:`compas_fab.robots.Configuration`
            The planning group's configuration.
        """
        if options is None:
            options = {}
        kwargs = {}
        kwargs['base_link'] = options['base_link']
        kwargs['frame'] = frame_WCF
        kwargs['group'] = group
        kwargs['start_configuration'] = start_configuration
        kwargs['avoid_collisions'] = options.get('avoid_collisions', True)
        kwargs['constraints'] = options.get('constraints')
        kwargs['attempts'] = options.get('attempts', 8)
        kwargs['attached_collision_meshes'] = options.get('attached_collision_meshes')

        kwargs['errback_name'] = 'errback'

        return await_callback(self.inverse_kinematics_async, **kwargs)

    def inverse_kinematics_async(self, callback, errback, base_link, frame, group,
                                 start_configuration, avoid_collisions=True,
                                 constraints=None, attempts=8, attached_collision_meshes=None):
        """Asynchronous handler of MoveIt IK service."""
        header = Header(frame_id=base_link)
        pose = Pose.from_frame(frame)
        pose_stamped = PoseStamped(header, pose)
        joint_state = JointState(
            name=start_configuration.joint_names, position=start_configuration.values, header=header)
        start_state = RobotState(
            joint_state, MultiDOFJointState(header=header))
        if attached_collision_meshes:
            for acm in attached_collision_meshes:
                aco = AttachedCollisionObject.from_attached_collision_mesh(acm)
                start_state.attached_collision_objects.append(aco)

        constraints = convert_constraints_to_rosmsg(constraints, header)

        ik_request = PositionIKRequest(group_name=group,
                                       robot_state=start_state,
                                       constraints=constraints,
                                       pose_stamped=pose_stamped,
                                       avoid_collisions=avoid_collisions,
                                       attempts=attempts)

        def convert_to_positions(response):
            callback((response.solution.joint_state.position, response.solution.joint_state.name))

        self.GET_POSITION_IK(self.ros_client, (ik_request, ), convert_to_positions, errback)

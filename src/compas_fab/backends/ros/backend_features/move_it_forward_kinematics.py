from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import ForwardKinematics
from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import GetPositionFKRequest
from compas_fab.backends.ros.messages import GetPositionFKResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    'MoveItForwardKinematics',
]


class MoveItForwardKinematics(ForwardKinematics):
    """Callable to calculate the robot's forward kinematic."""
    GET_POSITION_FK = ServiceDescription('/compute_fk',
                                         'GetPositionFK',
                                         GetPositionFKRequest,
                                         GetPositionFKResponse,
                                         validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def forward_kinematics(self, robot, configuration, group=None, options=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        robot : :class:`compas_fab.robots.Robot`
            The robot instance for which inverse kinematics is being calculated.
        configuration : :class:`compas_fab.robots.Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
        group : str, optional
            Unused parameter.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"base_link"``: (:obj:`str`) Name of the base link.
              Defaults to the model's root link.
            - ``"link"``: (:obj:`str`, optional) The name of the link to
              calculate the forward kinematics for. Defaults to the group's end
              effector link.
              Backwards compatibility note: if there's no ``link`` option, the
              planner will try also ``ee_link`` as fallback before defaulting
              to the end effector's link.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).
        """
        options = options or {}
        kwargs = {}
        kwargs['configuration'] = configuration
        kwargs['options'] = options
        kwargs['errback_name'] = 'errback'

        # Use base_link or fallback to model's root link
        options['base_link'] = options.get('base_link', robot.model.root.name)

        # Use selected link or default to group's end effector
        options['link'] = options.get('link', options.get('ee_link')) or robot.get_end_effector_link_name(group)
        if options['link'] not in robot.get_link_names(group):
            raise ValueError('Link name {} does not exist in planning group'.format(options['link']))

        return await_callback(self.forward_kinematics_async, **kwargs)

    def forward_kinematics_async(self, callback, errback,
                                 configuration, options):
        """Asynchronous handler of MoveIt FK service."""
        base_link = options['base_link']
        fk_link_names = [options['link']]

        header = Header(frame_id=base_link)
        joint_state = JointState(
            name=configuration.joint_names, position=configuration.values, header=header)
        robot_state = RobotState(
            joint_state, MultiDOFJointState(header=header))

        def convert_to_frame(response):
            callback(response.pose_stamped[0].pose.frame)

        self.GET_POSITION_FK(self.ros_client, (header, fk_link_names,
                                               robot_state), convert_to_frame, errback)

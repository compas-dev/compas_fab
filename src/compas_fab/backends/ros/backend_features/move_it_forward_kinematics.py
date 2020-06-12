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

    def forward_kinematics(self, configuration, group=None, options=None):
        """Calculate the robot's forward kinematic.

        Parameters
        ----------
        configuration : :class:`compas_fab.robots.Configuration`
            The full configuration to calculate the forward kinematic for. If no
            full configuration is passed, the zero-joint state for the other
            configurable joints is assumed.
        group : str, optional
            The planning group used for the calculation. Defaults to the robot's
            main planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - backend (:obj:`str`) :
                If `None` calculates fk with the client if it exists or with the robot model.
                If 'model' use the robot model to calculate fk. Anything else is open
                for implementation, possibly 'kdl', 'ikfast'
            - ee_link (:obj:`str`, optional) :
                The name of the link to calculate the forward kinematics for.
                Defaults to the group's end effector link.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).
        """
        kwargs = {}
        kwargs['configuration'] = configuration
        kwargs['group'] = group
        kwargs['options'] = options or {}

        kwargs['errback_name'] = 'errback'

        return await_callback(self.forward_kinematics_async, **kwargs)

    def forward_kinematics_async(self, callback, errback,
                                 configuration, group, options):
        """Asynchronous handler of MoveIt FK service."""
        base_link = options['base_link']
        header = Header(frame_id=base_link)
        ee_link = options['ee_link']
        fk_link_names = [ee_link]
        joint_state = JointState(
            name=configuration.joint_names, position=configuration.values, header=header)
        robot_state = RobotState(
            joint_state, MultiDOFJointState(header=header))

        def convert_to_frame(response):
            callback(response.pose_stamped[0].pose.frame)

        self.GET_POSITION_FK(self.ros_client, (header, fk_link_names,
                                               robot_state), convert_to_frame, errback)

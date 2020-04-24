from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback


from compas_fab.backends.ros.backend_features.helpers import validate_response
from compas_fab.backends.ros.messages import GetPositionFKRequest
from compas_fab.backends.ros.messages import GetPositionFKResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.planner_backend import ServiceDescription


class MoveItForwardKinematics(object):
    GET_POSITION_FK = ServiceDescription('/compute_fk',
                                         'GetPositionFK',
                                         GetPositionFKRequest,
                                         GetPositionFKResponse,
                                         validate_response)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def __call__(self, robot, configuration, group, ee_link):
        return self.forward_kinematics(robot, configuration, group, ee_link)

    def forward_kinematics(self, robot, configuration, group, ee_link):
        kwargs = {}
        kwargs['robot'] = robot
        kwargs['configuration'] = configuration
        kwargs['group'] = group
        kwargs['ee_link'] = ee_link

        kwargs['errback_name'] = 'errback'

        return await_callback(self.forward_kinematics_async, **kwargs)

    def forward_kinematics_async(self, callback, errback, robot, configuration,
                                 group, ee_link):
        """Asynchronous handler of MoveIt FK service."""
        base_link = robot.model.root.name
        header = Header(frame_id=base_link)
        fk_link_names = [ee_link]
        joint_state = JointState(
            name=configuration.joint_names, position=configuration.values, header=header)
        robot_state = RobotState(
            joint_state, MultiDOFJointState(header=header))

        def convert_to_frame(response):
            callback(response.pose_stamped[0].pose.frame)

        self.GET_POSITION_FK(self.ros_client, (header, fk_link_names,
                                               robot_state), convert_to_frame, errback)

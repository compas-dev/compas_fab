from roslibpy import Ros
from roslibpy import Service
from roslibpy import ServiceRequest

from compas.geometry import Frame

from compas_fab.backends.ros import Header
from compas_fab.backends.ros import Pose
from compas_fab.backends.ros import PoseStamped
from compas_fab.backends.ros import JointState
from compas_fab.backends.ros import MultiDOFJointState
from compas_fab.backends.ros import RobotState
from compas_fab.backends.ros import PositionIKRequest
from compas_fab.backends.ros import GetPositionIKRequest
from compas_fab.backends.ros import GetPositionIKResponse
from compas_fab.backends.ros import GetPositionFKRequest
from compas_fab.backends.ros import GetPositionFKResponse
from compas_fab.backends.ros import GetCartesianPathRequest
from compas_fab.backends.ros import GetCartesianPathResponse

class Client(Ros):

    def inverse_kinematics(self, frame, base_link, group, joint_names, joint_positions):

        header = Header(frame_id=base_link)
        pose = Pose.from_frame(frame)
        pose_stamped = PoseStamped(header, pose)
        joint_state = JointState(name=joint_names, position=joint_positions, header=header)

        multi_dof_joint_state = MultiDOFJointState(header=header)
        start_state = RobotState(joint_state, multi_dof_joint_state)
        ik_request = PositionIKRequest(group_name=group,
                                       robot_state=start_state,
                                       pose_stamped=pose_stamped,
                                       avoid_collisions=True)
        reqmsg = GetPositionIKRequest(ik_request)



    def forward_kinematics(self, configuration, base_link, group, joint_names, ee_link):

        header = Header(frame_id=base_link)
        fk_link_names = [ee_link]
        joint_state = JointState(name=joint_names, position=joint_positions, header=header)
        multi_dof_joint_state = MultiDOFJointState(header=header)
        robot_state = RobotState(joint_state, multi_dof_joint_state)
        reqmsg = GetPositionFKRequest(header, fk_link_names, robot_state)


        def receive_message(msg):
            response = GetPositionFKResponse.from_msg(msg)
            if response.error_code == MoveItErrorCodes.SUCCESS:
                frames = [ps.pose.frame for ps in response.pose_stamped]
            print(response.error_code.human_readable)


        srv = roslibpy.Service(ros, '/compute_fk', 'GetPositionFK')
        request = roslibpy.ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)


    def compute_cartesian_path(self, frames, base_link, ee_link, group,
                               joint_names, joint_positions, max_step,
                               avoid_collisions):

        header = Header(frame_id=base_link)
        waypoints = [Pose.from_frame(frame) for frame in frames]
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        multi_dof_joint_state = MultiDOFJointState(header=header)
        start_state = RobotState(joint_state=joint_state, multi_dof_joint_state=multi_dof_joint_state)
        reqmsg = GetCartesianPathRequest(header=header,
                                         start_state=start_state,
                                         group_name=group,
                                         link_name=ee_link,
                                         waypoints=waypoints,
                                         max_step=float(max_step)/1000.,
                                         avoid_collisions=bool(avoid_collisions))
        def receive_message(msg):
            response = GetCartesianPathResponse.from_msg(msg)

        srv = Service(self, '/compute_cartesian_path', 'GetCartesianPath')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)



    def send_frame(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_configuration(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

    def send_trajectory(self):
        #(check service name with ros)
        self.ensure_client()
        raise NotImplementedError

from compas.geometry import Frame
from roslibpy import Ros
from roslibpy import Service
from roslibpy import ServiceRequest
from roslibpy import Message
from roslibpy.actionlib import ActionClient
from roslibpy.actionlib import Goal

from compas_fab.backends.ros import GetCartesianPathRequest
from compas_fab.backends.ros import GetCartesianPathResponse
from compas_fab.backends.ros import GetPositionFKRequest
from compas_fab.backends.ros import GetPositionFKResponse
from compas_fab.backends.ros import GetPositionIKRequest
from compas_fab.backends.ros import GetPositionIKResponse
from compas_fab.backends.ros import Header
from compas_fab.backends.ros import JointState
from compas_fab.backends.ros import MultiDOFJointState
from compas_fab.backends.ros import Pose
from compas_fab.backends.ros import PoseStamped
from compas_fab.backends.ros import PositionIKRequest
from compas_fab.backends.ros import RobotState
from compas_fab.backends.ros import MoveItErrorCodes
from compas_fab.backends.ros import FollowJointTrajectoryResult
from compas_fab.backends.ros import FollowJointTrajectoryGoal
from compas_fab.backends.ros import JointTrajectoryPoint
from compas_fab.backends.ros import JointTrajectory
from compas_fab.backends.ros import Time

from compas_fab.backends.ros import DirectUrActionClient
from compas_fab.backends.ros.messages.direct_ur import URPose
from compas_fab.backends.ros.messages.direct_ur import URPoseTrajectoryPoint
from compas_fab.backends.ros.messages.direct_ur import URMovej
from compas_fab.backends.ros.messages.direct_ur import URMovel
from compas_fab.backends.ros.messages.direct_ur import URGoal


__all__ = [
    'RosClient'
]


class RosClient(Ros):

    def inverse_kinematics(self, callback, frame, base_link, group, joint_names, joint_positions):

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

        def receive_message(msg):
            response = GetPositionIKResponse.from_msg(msg)
            callback(response)
            """
            if response.error_code == MoveItErrorCodes.SUCCESS:
                configuration = response.solution.joint_state.position
                print(configuration)
            print(response.error_code.human_readable)
            """

        srv = Service(self, '/compute_ik', 'GetPositionIK')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)


    def forward_kinematics(self, callback, configuration, base_link, group, joint_names, ee_link):
        
        joint_positions = configuration.values
        header = Header(frame_id=base_link)
        fk_link_names = [ee_link]
        joint_state = JointState(name=joint_names, position=joint_positions, header=header)
        multi_dof_joint_state = MultiDOFJointState(header=header)
        robot_state = RobotState(joint_state, multi_dof_joint_state)
        reqmsg = GetPositionFKRequest(header, fk_link_names, robot_state)

        def receive_message(msg):
            response = GetPositionFKResponse.from_msg(msg)
            #if response.error_code == MoveItErrorCodes.SUCCESS:
            #    frames = [ps.pose.frame for ps in response.pose_stamped]
            #print(response.error_code.human_readable)
            callback(response)

        srv = Service(self, '/compute_fk', 'GetPositionFK')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)

    def compute_cartesian_path(self, callback, frames, base_link, ee_link, group,
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
            callback(response)

        srv = Service(self, '/compute_cartesian_path', 'GetCartesianPath')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)
    
    def follow_configurations(self, callback, joint_names, configurations, timesteps, timeout=None):

        if len(configurations) != len(timesteps):
            raise ValueError("%d configurations must have %d timesteps, but %d given." % (len(configurations), len(timesteps), len(timesteps)))

        if not timeout:
            timeout = timesteps[-1] * 1000 * 2

        points = []
        num_joints = len(configurations[0].values)
        for config, time in zip(configurations, timesteps):
            pt = JointTrajectoryPoint(positions=config.values, velocities=[0]*num_joints, time_from_start=Time(secs=(time)))
            points.append(pt)

        joint_trajectory = JointTrajectory(Header(), joint_names, points) # specify header necessary?
        self.follow_joint_trajectory(callback, joint_trajectory, timeout)
    

    def follow_joint_trajectory(self, callback, joint_trajectory, timeout=3000):
        """Follow the joint trajectory as computed by Moveit Planner.

        Args:
            joint_trajectory (JointTrajectory)
        """

        goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory)

        def handle_result(msg, client):
            result = FollowJointTrajectoryResult.from_msg(msg)
            callback(result)
            #print(result.human_readable)

        action_client = ActionClient(self, '/follow_joint_trajectory', 
                       'control_msgs/FollowJointTrajectory', timeout)
        goal = Goal(action_client, Message(goal.msg))

        goal.on('result', lambda result: handle_result(result, action_client))
        goal.on('feedback', lambda feedback: print(feedback))
        goal.on('timeout', lambda: print('TIMEOUT'))
        action_client.on('timeout', lambda: print('CLIENT TIMEOUT'))
        goal.send(60000)

    
    def direct_ur_movel(self, callback, frames, acceleration=None, velocity=None, time=None, radius=None):

        action_client = DirectUrActionClient(self, timeout=50000)

        script_lines = []
        for frame in frames:        
            ptp = URPoseTrajectoryPoint(URPose.from_frame(frame), acceleration, velocity, time, radius)
            #move = URMovej(ptp)
            move = URMovel(ptp)
            script_lines.append(move)

        urgoal = URGoal(script_lines)

        goal = Goal(action_client, Message(urgoal.msg))
        action_client.on('timeout', lambda: print('CLIENT TIMEOUT'))
        # goal.on('feedback', lambda feedback: print(feedback))
        goal.on('result', callback)
        action_client.send_goal(goal)



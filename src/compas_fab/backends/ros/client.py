from __future__ import print_function

from compas.geometry import Frame
from roslibpy import Message
from roslibpy import Ros
from roslibpy import Service
from roslibpy import ServiceRequest
from roslibpy.actionlib import ActionClient
from roslibpy.actionlib import Goal

from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.ros import DirectUrActionClient
from compas_fab.backends.ros import FollowJointTrajectoryGoal
from compas_fab.backends.ros import FollowJointTrajectoryResult
from compas_fab.backends.ros import GetCartesianPathRequest
from compas_fab.backends.ros import GetCartesianPathResponse
from compas_fab.backends.ros import GetPositionFKRequest
from compas_fab.backends.ros import GetPositionFKResponse
from compas_fab.backends.ros import GetPositionIKRequest
from compas_fab.backends.ros import GetPositionIKResponse
from compas_fab.backends.ros import MotionPlanRequest
from compas_fab.backends.ros import MotionPlanResponse
from compas_fab.backends.ros import Header
from compas_fab.backends.ros import JointState
from compas_fab.backends.ros import JointTrajectory
from compas_fab.backends.ros import JointTrajectoryPoint
from compas_fab.backends.ros import MoveItErrorCodes
from compas_fab.backends.ros import MultiDOFJointState
from compas_fab.backends.ros import Pose
from compas_fab.backends.ros import PoseStamped
from compas_fab.backends.ros import PositionIKRequest
from compas_fab.backends.ros import RobotState
from compas_fab.backends.ros import Time
from compas_fab.backends.ros import PositionConstraint
from compas_fab.backends.ros import OrientationConstraint
from compas_fab.backends.ros import JointConstraint
from compas_fab.backends.ros import SolidPrimitive
from compas_fab.backends.ros import Quaternion
from compas_fab.backends.ros import Constraints
from compas_fab.backends.ros import TrajectoryConstraints

from compas_fab.backends.ros.messages.direct_ur import URGoal
from compas_fab.backends.ros.messages.direct_ur import URMovej
from compas_fab.backends.ros.messages.direct_ur import URMovel
from compas_fab.backends.ros.messages.direct_ur import URPose
from compas_fab.backends.ros.messages.direct_ur import URPoseTrajectoryPoint

__all__ = [
    'RosClient',
    'RosError',
]


class RosError(BackendError):
    """Wraps an exception that occurred on the communication with ROS."""
    pass


class RosClient(Ros):
    """Interface to use ROS as backend via the **rosbridge**.

    The connection is managed by ``roslibpy``.

    Parameters
    ----------
    host : :obj:`str`
        ROS bridge host. Defaults to ``localhost``.
    port : :obj:`int`
        Port of the ROS Bridge. Defaults to ``9090``.
    is_secure : :obj:`bool`
        ``True`` to indicate it should use a secure web socket, otherwise ``False``.

    Examples
    --------

    >>> from compas_fab.backends import RosClient
    >>> client = RosClient()
    >>> def hello_ros():
    >>>     print('Connected: %s' % client.is_connected)
    >>>     client.terminate()
    >>> client.on_ready(hello_ros)
    >>> client.run_forever()
    Connected: True

    Note
    ----
    For more examples, check out the :ref:`ROS examples page <ros_examples>`.
    """

    def __init__(self, host='localhost', port=9090, is_secure=False):
        super(RosClient, self).__init__(host, port, is_secure)

    def inverse_kinematics(self, callback_result, frame, base_link, group, joint_names, joint_positions):

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
            callback_result(response)
            """
            if response.error_code == MoveItErrorCodes.SUCCESS:
                configuration = response.solution.joint_state.position
            print(response.error_code.human_readable)
            """

        srv = Service(self, '/compute_ik', 'GetPositionIK')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)


    def forward_kinematics(self, callback_result, joint_positions, base_link, group, joint_names, ee_link):

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
            callback_result(response)

        srv = Service(self, '/compute_fk', 'GetPositionFK')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)

    def compute_cartesian_path(self, callback_result, frames, base_link, ee_link, group,
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
                                         max_step=float(max_step),
                                         avoid_collisions=bool(avoid_collisions))

        def receive_message(msg):
            response = GetCartesianPathResponse.from_msg(msg)
            callback_result(response)

        srv = Service(self, '/compute_cartesian_path', 'GetCartesianPath')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)
    
    def motion_plan(self, callback_result, frame, base_link, ee_link, group,
                    joint_names, joint_positions, tolerance_position, tolerance_angle, 
                    path_constraints=None, trajectory_constraints=None, 
                    planner_id=None):
        """
        """
        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: path_contraints, planner_id?
        # TODO: if list of frames (goals) => receive multiple solutions?

        header = Header(frame_id=base_link)
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        multi_dof_joint_state = MultiDOFJointState(header=header)
        start_state = RobotState(joint_state=joint_state, multi_dof_joint_state=multi_dof_joint_state)
  
        pose = Pose.from_frame(frame)

        pcm = PositionConstraint(header=header, link_name=ee_link)
        pcm.target_point_offset.x = 0.
        pcm.target_point_offset.y = 0.
        pcm.target_point_offset.z = 0.
        bv = SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[tolerance_position])
        pcm.constraint_region.primitives = [bv]
        pcm.constraint_region.primitive_poses = [Pose(pose.position, Quaternion(0,0,0,1))]

        ocm = OrientationConstraint(header=header, link_name=ee_link)
        ocm.orientation = Quaternion.from_frame(frame)
        ocm.absolute_x_axis_tolerance = tolerance_angle
        ocm.absolute_y_axis_tolerance = tolerance_angle
        ocm.absolute_z_axis_tolerance = tolerance_angle
        
        goal_constraints = [Constraints(position_constraints=[pcm], orientation_constraints=[ocm])]

        # TODO: add to parameters ...
        if not path_constraints:
            path_constraints = Constraints()
        if not trajectory_constraints:
            trajectory_constraints = TrajectoryConstraints()
            
        reqmsg = MotionPlanRequest(start_state=start_state, 
                                   goal_constraints=goal_constraints, 
                                   path_constraints=path_constraints,
                                   trajectory_constraints=trajectory_constraints,
                                   planner_id=planner_id,
                                   group_name=group)
        
        def receive_message(msg):
            response = MotionPlanResponse.from_msg(msg)
            callback_result(response)

        srv = Service(self, '/plan_kinematic_path', 'GetMotionPlan')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)
    

    def motion_plan_joint_positions_goal(self, callback_result, 
                    joint_positions_goal, joint_names_goal, tolerances, 
                    base_link, group, joint_names, joint_positions,
                    path_constraints=None, trajectory_constraints=None, 
                    planner_id=None):
        """
        """
        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html

        header = Header(frame_id=base_link)
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        multi_dof_joint_state = MultiDOFJointState(header=header)
        start_state = RobotState(joint_state=joint_state, multi_dof_joint_state=multi_dof_joint_state)

        joint_constraints = []
        for position, joint_name, tolerance in zip(joint_positions_goal, joint_names_goal, tolerances):
            jcm = JointConstraint(joint_name, position, tolerance, tolerance)
            joint_constraints.append(jcm)
  
        goal_constraints = [Constraints(joint_constraints=joint_constraints)]

        # TODO: add to parameters ...
        if not path_constraints:
            path_constraints = Constraints()
        if not trajectory_constraints:
            trajectory_constraints = TrajectoryConstraints()
            
        reqmsg = MotionPlanRequest(start_state=start_state, 
                                   goal_constraints=goal_constraints, 
                                   path_constraints=path_constraints,
                                   trajectory_constraints=trajectory_constraints,
                                   planner_id=planner_id,
                                   group_name=group)
        
        def receive_message(msg):
            response = MotionPlanResponse.from_msg(msg)
            callback_result(response)

        srv = Service(self, '/plan_kinematic_path', 'GetMotionPlan')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)


    def follow_configurations(self, callback_result, joint_names, configurations, timesteps, timeout=None):

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
        self.follow_joint_trajectory(callback_result, joint_trajectory, timeout)

    def follow_joint_trajectory(self, callback_result, joint_trajectory, timeout=3000):
        """Follow the joint trajectory as computed by Moveit Planner.

        Args:
            joint_trajectory (JointTrajectory)
        """

        goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory)

        def handle_result(msg, client):
            result = FollowJointTrajectoryResult.from_msg(msg)
            callback_result(result)
            #print(result.human_readable)

        action_client = ActionClient(self, '/follow_joint_trajectory',
                       'control_msgs/FollowJointTrajectoryAction', timeout)
        goal = Goal(action_client, Message(goal.msg))

        goal.on('result', lambda result: handle_result(result, action_client))
        goal.on('feedback', lambda feedback: print(feedback))
        goal.on('timeout', lambda: print('TIMEOUT'))
        action_client.on('timeout', lambda: print('CLIENT TIMEOUT'))
        goal.send(60000)


    def direct_ur_movel(self, callback_result, frames, acceleration=None, velocity=None, time=None, radius=None):

        action_client = DirectUrActionClient(self, timeout=50000)

        script_lines = []
        for frame in frames:
            ptp = URPoseTrajectoryPoint(URPose.from_frame(frame), acceleration, velocity, time, radius)
            move = URMovel(ptp)
            script_lines.append(move)

        urgoal = URGoal(script_lines)
        
        goal = Goal(action_client, Message(urgoal.msg))
        action_client.on('timeout', lambda: print('CLIENT TIMEOUT'))
        # goal.on('feedback', lambda feedback: print(feedback))
        goal.on('result', callback_result)
        action_client.send_goal(goal)

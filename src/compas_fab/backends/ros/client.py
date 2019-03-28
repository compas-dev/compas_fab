from __future__ import print_function

import functools

from compas.datastructures import mesh_quads_to_triangles
from compas.geometry import Frame
from compas.utilities import await_callback
from roslibpy import Message
from roslibpy import Ros
from roslibpy import Service
from roslibpy import ServiceRequest
from roslibpy import Topic
from roslibpy.actionlib import ActionClient
from roslibpy.actionlib import Goal

from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.tasks import CancellableTask
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import Constraints
from compas_fab.backends.ros.messages import FollowJointTrajectoryGoal
from compas_fab.backends.ros.messages import FollowJointTrajectoryResult
from compas_fab.backends.ros.messages import GetCartesianPathRequest
from compas_fab.backends.ros.messages import GetCartesianPathResponse
from compas_fab.backends.ros.messages import GetPlanningSceneRequest
from compas_fab.backends.ros.messages import GetPlanningSceneResponse
from compas_fab.backends.ros.messages import GetPositionFKRequest
from compas_fab.backends.ros.messages import GetPositionFKResponse
from compas_fab.backends.ros.messages import GetPositionIKRequest
from compas_fab.backends.ros.messages import GetPositionIKResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointConstraint
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import JointTrajectory
from compas_fab.backends.ros.messages import JointTrajectoryPoint
from compas_fab.backends.ros.messages import Mesh
from compas_fab.backends.ros.messages import MotionPlanRequest
from compas_fab.backends.ros.messages import MotionPlanResponse
from compas_fab.backends.ros.messages import MoveItErrorCodes
from compas_fab.backends.ros.messages import MultiDOFJointState
from compas_fab.backends.ros.messages import OrientationConstraint
from compas_fab.backends.ros.messages import PlanningSceneComponents
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.messages import PoseStamped
from compas_fab.backends.ros.messages import PositionConstraint
from compas_fab.backends.ros.messages import PositionIKRequest
from compas_fab.backends.ros.messages import Quaternion
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages import SolidPrimitive
from compas_fab.backends.ros.messages import Time
from compas_fab.backends.ros.messages import TrajectoryConstraints

__all__ = [
    'RosClient',
    'RosError',
]


def validated_response(func, *args, **kwargs):
    """Decorator to wrap a function that returns a ROS response
    and raise an exception if the response indicates an error condition."""
    @functools.wraps(func)
    def validated_func(*args, **kwargs):
        response = func(*args, **kwargs)

        if response.error_code != MoveItErrorCodes.SUCCESS:
            raise RosError(response.error_code.human_readable,
                           int(response.error_code), response)

        return response

    return validated_func


class CancellableRosAction(CancellableTask):
    def __init__(self, goal):
        self.goal = goal

    def cancel(self):
        self.goal.cancel()


class ServiceDescription(object):
    """Internal class to simplify service call code."""

    def __init__(self, name, service_type, request_class=None, response_class=None):
        self.name = name
        self.type = service_type
        self.request_class = request_class or eval(service_type + 'Request')
        self.response_class = response_class or eval(service_type + 'Response')

    def call(self, client, request, callback, errback):
        def inner_handler(response_msg):
            callback(self.response_class.from_msg(response_msg))

        if isinstance(request, tuple):
            request_msg = self.request_class(*request)
        else:
            request_msg = self.request_class(**request)

        srv = Service(client, self.name, self.type)
        srv.call(ServiceRequest(request_msg.msg), callback=inner_handler, errback=errback)

    def __call__(self, client, request, callback, errback):
        return self.call(client, request, callback, errback)


class RosError(BackendError):
    """Wraps an exception that occurred on the communication with ROS."""

    def __init__(self, message, error_code, ros_msg):
        super(RosError, self).__init__('Error code: ' +
                                       str(error_code) +
                                       '; ' + message)
        self.error_code = error_code
        self.ros_msg = ros_msg


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
    >>> client.run()
    >>> print('Connected: %s' % client.is_connected)
    Connected: True

    Note
    ----
    For more examples, check out the :ref:`ROS examples page <ros_examples>`.
    """
    GET_POSITION_IK = ServiceDescription('/compute_ik', 'GetPositionIK')
    GET_POSITION_FK = ServiceDescription('/compute_fk', 'GetPositionFK')
    GET_CARTESIAN_PATH = ServiceDescription('/compute_cartesian_path', 'GetCartesianPath')
    GET_MOTION_PLAN = ServiceDescription('/plan_kinematic_path', 'GetMotionPlan', MotionPlanRequest, MotionPlanResponse)

    def __init__(self, host='localhost', port=9090, is_secure=False):
        super(RosClient, self).__init__(host, port, is_secure)

    @validated_response
    def inverse_kinematics(self, frame, base_link, group,
                           joint_names, joint_positions, avoid_collisions=True,
                           constraints=None, attempts=8):
        kwargs = {}
        kwargs['frame'] = frame
        kwargs['base_link'] = base_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_positions'] = joint_positions
        kwargs['avoid_collisions'] = avoid_collisions
        kwargs['constraints'] = constraints
        kwargs['attempts'] = attempts

        kwargs['errback_name'] = 'errback'

        return await_callback(self.inverse_kinematics_async, **kwargs)

    def inverse_kinematics_async(self, callback, errback, frame, base_link, group,
                                 joint_names, joint_positions, avoid_collisions=True,
                                 constraints=None, attempts=8):
        """
        """
        header = Header(frame_id=base_link)
        pose = Pose.from_frame(frame)
        pose_stamped = PoseStamped(header, pose)
        joint_state = JointState(name=joint_names, position=joint_positions, header=header)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))

        ik_request = PositionIKRequest(group_name=group,
                                       robot_state=start_state,
                                       constraints=constraints,
                                       pose_stamped=pose_stamped,
                                       avoid_collisions=avoid_collisions,
                                       attempts=attempts)

        self.GET_POSITION_IK(self, (ik_request, ), callback, errback)

    @validated_response
    def forward_kinematics(self, joint_positions, base_link, group, joint_names, ee_link):
        kwargs = {}
        kwargs['joint_positions'] = joint_positions
        kwargs['base_link'] = base_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['ee_link'] = ee_link

        kwargs['errback_name'] = 'errback'

        return await_callback(self.forward_kinematics_async, **kwargs)

    def forward_kinematics_async(self, callback, errback, joint_positions, base_link,
                                 group, joint_names, ee_link):
        """
        """
        header = Header(frame_id=base_link)
        fk_link_names = [ee_link]
        joint_state = JointState(name=joint_names, position=joint_positions, header=header)
        robot_state = RobotState(joint_state, MultiDOFJointState(header=header))

        self.GET_POSITION_FK(self, (header, fk_link_names, robot_state), callback, errback)

    @validated_response
    def plan_cartesian_motion(self, frames, base_link,
                               ee_link, group, joint_names, joint_positions,
                               max_step, avoid_collisions, path_constraints,
                               attached_collision_object):
        kwargs = {}
        kwargs['frames'] = frames
        kwargs['base_link'] = base_link
        kwargs['ee_link'] = ee_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_positions'] = joint_positions
        kwargs['max_step'] = max_step
        kwargs['avoid_collisions'] = avoid_collisions
        kwargs['path_constraints'] = path_constraints
        kwargs['attached_collision_object'] = attached_collision_object

        kwargs['errback_name'] = 'errback'

        return await_callback(self.plan_cartesian_motion_async, **kwargs)

    def plan_cartesian_motion_async(self, callback, errback, frames, base_link,
                                     ee_link, group, joint_names, joint_positions,
                                     max_step, avoid_collisions, path_constraints,
                                     attached_collision_object):
        """
        """
        header = Header(frame_id=base_link)
        waypoints = [Pose.from_frame(frame) for frame in frames]
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))
        if attached_collision_object:
            start_state.attached_collision_objects = [attached_collision_object]

        request = dict(header=header,
                       start_state=start_state,
                       group_name=group,
                       link_name=ee_link,
                       waypoints=waypoints,
                       max_step=float(max_step),
                       avoid_collisions=bool(avoid_collisions),
                       path_constraints=path_constraints)

        self.GET_CARTESIAN_PATH(self, request, callback, errback)

    @validated_response
    def plan_motion(self, goal_constraints, base_link, ee_link, group, 
                    joint_names, joint_positions, path_constraints=None,
                    trajectory_constraints=None, planner_id='', 
                    num_planning_attempts=8, allowed_planning_time=2., 
                    max_velocity_scaling_factor=1., 
                    max_acceleration_scaling_factor=1.,
                    attached_collision_object=None,
                    workspace_parameters=None):
        
        kwargs = {}
        kwargs['goal_constraints'] = goal_constraints
        kwargs['base_link'] = base_link
        kwargs['ee_link'] = ee_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_positions'] = joint_positions
        kwargs['path_constraints'] = path_constraints
        kwargs['trajectory_constraints'] = trajectory_constraints
        kwargs['planner_id'] = planner_id
        kwargs['num_planning_attempts'] = num_planning_attempts
        kwargs['allowed_planning_time'] = allowed_planning_time
        kwargs['max_velocity_scaling_factor'] = max_velocity_scaling_factor
        kwargs['max_acceleration_scaling_factor'] = max_acceleration_scaling_factor
        kwargs['attached_collision_object'] = attached_collision_object
        kwargs['workspace_parameters'] = workspace_parameters

        kwargs['errback_name'] = 'errback'

        return await_callback(self.plan_motion_async, **kwargs)
    
    def plan_motion_async(self, callback, errback, goal_constraints, base_link,
                          ee_link, group, joint_names, joint_positions,
                          path_constraints=None, trajectory_constraints=None,
                          planner_id='', num_planning_attempts=8,
                          allowed_planning_time=2., 
                          max_velocity_scaling_factor=1.,
                          max_acceleration_scaling_factor=1.,
                          attached_collision_object=None,
                          workspace_parameters=None):
        """
        """
        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: if list of frames (goals) => receive multiple solutions?

        header = Header(frame_id=base_link)
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))
        if attached_collision_object:
            start_state.attached_collision_objects = [attached_collision_object]

        # goal constraints
        constraints = Constraints()
        for c in goal_constraints:
            if c.type == c.JOINT:
                constraints.joint_constraints.append(JointConstraint.from_joint_constraint(c))
            elif c.type == c.POSITION:
                constraints.position_constraints.append(PositionConstraint.from_position_constraint(header, c))
            elif c.type == c.ORIENTATION:
                constraints.orientation_constraints.append(OrientationConstraint.from_orientation_constraint(header, c))
            else:
                raise NotImplementedError
        goal_constraints = [constraints]

        # path constraints
        if path_constraints:
            constraints = Constraints()
            for c in path_constraints:
                if c.type == c.JOINT:
                    constraints.joint_constraints.append(JointConstraint.from_joint_constraint(c))
                elif c.type == c.POSITION:
                    constraints.position_constraints.append(PositionConstraint.from_position_constraint(header, c))
                elif c.type == c.ORIENTATION:
                    constraints.orientation_constraints.append(OrientationConstraint.from_orientation_constraint(header, c))
                else:
                    raise NotImplementedError
            path_constraints = constraints

        request = dict(start_state=start_state,
                       goal_constraints=goal_constraints,
                       path_constraints=path_constraints,
                       trajectory_constraints=trajectory_constraints,
                       planner_id=planner_id,
                       group_name=group,
                       num_planning_attempts=num_planning_attempts,
                       allowed_planning_time=allowed_planning_time,
                       max_velocity_scaling_factor=max_velocity_scaling_factor,
                       max_acceleration_scaling_factor=max_velocity_scaling_factor)
        # workspace_parameters=workspace_parameters

        self.GET_MOTION_PLAN(self, request, callback, errback)

    # ==========================================================================
    # collision objects
    # ==========================================================================

    def add_collision_mesh(self, collision_mesh):
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self.collision_object(co, CollisionObject.ADD)

    def remove_collision_mesh(self, id):
        co = CollisionObject()
        co.id = id
        self.collision_object(co, CollisionObject.REMOVE)

    def append_collision_mesh(self, collision_mesh):
        co = CollisionObject.from_collision_mesh(collision_mesh)
        self.collision_object(co, CollisionObject.APPEND)

    def collision_object(self, collision_object, operation=CollisionObject.ADD):
        """
        """            
        collision_object.operation = operation
        topic = Topic(self, '/collision_object', 'moveit_msgs/CollisionObject')
        topic.publish(collision_object.msg)

    def add_attached_collision_mesh(self, attached_collision_mesh):
        """
        """
        aco = AttachedCollisionObject.from_attached_collision_mesh(attached_collision_mesh)
        self.attached_collision_object(aco, operation=CollisionObject.ADD)
    
    def remove_attached_collision_mesh(self, id):
        """
        """
        aco = AttachedCollisionObject()
        aco.object.id = id
        return self.attached_collision_object(aco, operation=CollisionObject.REMOVE)

    def attached_collision_object(self, attached_collision_object, operation=CollisionObject.ADD):
        """
        """
        attached_collision_object.object.operation = operation
        topic = Topic(self, '/attached_collision_object', 'moveit_msgs/AttachedCollisionObject')
        topic.publish(attached_collision_object.msg)
    
    # ==========================================================================
    # executing
    # ==========================================================================

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

        joint_trajectory = JointTrajectory(Header(), joint_names, points)  # specify header necessary?
        return self.follow_joint_trajectory(callback, joint_trajectory, timeout)

    def follow_joint_trajectory(self, joint_trajectory, callback=None, errback=None, feedback_callback=None, timeout=60000):
        """Follow the joint trajectory as computed by MoveIt planner.

        Parameters
        ----------
        joint_trajectory : JointTrajectory
            Joint trajectory message as computed by MoveIt planner
        callback : callable
            Function to be invoked when the goal is completed, requires
            one positional parameter ``result``.
        errback : callable
            Function to be invoked in case of error or timeout, requires
            one position parameter ``exception``.
        feedback_callback : callable
            Function to be invoked during execution to provide feedback.
        timeout : int
            Timeout for goal completion in milliseconds.

        Returns
        -------
        :class:`CancellableTask`
            An instance of a cancellable tasks.
        """

        trajectory_goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory)

        def handle_result(msg, client):
            result = FollowJointTrajectoryResult.from_msg(msg)
            callback(result)

        def handle_failure(error):
            errback(error)

        connection_timeout = 3000
        action_client = ActionClient(self, '/joint_trajectory_action',  # '/follow_joint_trajectory',
                                     'control_msgs/FollowJointTrajectoryAction', connection_timeout)

        goal = Goal(action_client, Message(trajectory_goal.msg))

        if callback:
            goal.on('result', lambda result: handle_result(result, action_client))

        if feedback_callback:
            goal.on('feedback', feedback_callback)

        if errback:
            goal.on('timeout', lambda: handle_failure(RosError("Action Goal timeout", -1, None)))
            action_client.on('timeout', lambda: handle_failure(RosError("Actionlib client timeout", -1, None)))

        goal.send(timeout=timeout)

        return CancellableRosAction(goal)

    def get_planning_scene(self, callback, components):
        """
        """
        reqmsg = GetPlanningSceneRequest(PlanningSceneComponents(components))

        def receive_message(msg):
            response = GetPlanningSceneResponse.from_msg(msg)
            callback(response)

        srv = Service(self, '/get_planning_scene', 'moveit_msgs/GetPlanningScene')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)

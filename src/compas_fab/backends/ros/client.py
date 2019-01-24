from __future__ import print_function

import functools

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
                           constraints=None):
        kwargs = {}
        kwargs['frame'] = frame
        kwargs['base_link'] = base_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_positions'] = joint_positions
        kwargs['avoid_collisions'] = avoid_collisions
        kwargs['constraints'] = constraints

        kwargs['errback_name'] = 'errback'

        return await_callback(self.inverse_kinematics_async, **kwargs)

    def inverse_kinematics_async(self, callback, errback, frame, base_link, group,
                                 joint_names, joint_positions, avoid_collisions=True,
                                 constraints=None):
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
                                       avoid_collisions=avoid_collisions)

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
    def compute_cartesian_path(self, frames, base_link,
                               ee_link, group, joint_names, joint_positions,
                               max_step, avoid_collisions):
        kwargs = {}
        kwargs['frames'] = frames
        kwargs['base_link'] = base_link
        kwargs['ee_link'] = ee_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_positions'] = joint_positions
        kwargs['max_step'] = max_step
        kwargs['avoid_collisions'] = avoid_collisions

        kwargs['errback_name'] = 'errback'

        return await_callback(self.compute_cartesian_path_async, **kwargs)

    def compute_cartesian_path_async(self, callback, errback, frames, base_link,
                                     ee_link, group, joint_names, joint_positions,
                                     max_step, avoid_collisions):
        """
        """
        header = Header(frame_id=base_link)
        waypoints = [Pose.from_frame(frame) for frame in frames]
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))

        request = dict(header=header,
                       start_state=start_state,
                       group_name=group,
                       link_name=ee_link,
                       waypoints=waypoints,
                       max_step=float(max_step),
                       avoid_collisions=bool(avoid_collisions))

        self.GET_CARTESIAN_PATH(self, request, callback, errback)

    @validated_response
    def motion_plan_goal_frame(self, frame, base_link, ee_link,
                               group, joint_names, joint_positions,
                               tolerance_position, tolerance_angle,
                               path_constraints=None,
                               trajectory_constraints=None,
                               planner_id='', num_planning_attempts=8,
                               allowed_planning_time=2.,
                               max_velocity_scaling_factor=1.,
                               max_acceleration_scaling_factor=1.):
        kwargs = {}
        kwargs['frame'] = frame
        kwargs['base_link'] = base_link
        kwargs['ee_link'] = ee_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_positions'] = joint_positions
        kwargs['tolerance_position'] = tolerance_position
        kwargs['tolerance_angle'] = tolerance_angle
        kwargs['path_constraints'] = path_constraints
        kwargs['trajectory_constraints'] = trajectory_constraints
        kwargs['planner_id'] = planner_id
        kwargs['num_planning_attempts'] = num_planning_attempts
        kwargs['allowed_planning_time'] = allowed_planning_time
        kwargs['max_velocity_scaling_factor'] = max_velocity_scaling_factor
        kwargs['max_acceleration_scaling_factor'] = max_acceleration_scaling_factor

        kwargs['errback_name'] = 'errback'

        return await_callback(self.motion_plan_goal_frame_async, **kwargs)

    def motion_plan_goal_frame_async(self, callback, errback, frame, base_link, ee_link,
                                     group, joint_names, joint_positions,
                                     tolerance_position, tolerance_angle,
                                     path_constraints=None,
                                     trajectory_constraints=None,
                                     planner_id='', num_planning_attempts=8,
                                     allowed_planning_time=2.,
                                     max_velocity_scaling_factor=1.,
                                     max_acceleration_scaling_factor=1.):
        """
        """
        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html
        # TODO: if list of frames (goals) => receive multiple solutions?

        header = Header(frame_id=base_link)
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))

        pose = Pose.from_frame(frame)

        pcm = PositionConstraint(header=header, link_name=ee_link)
        pcm.target_point_offset.x = 0.
        pcm.target_point_offset.y = 0.
        pcm.target_point_offset.z = 0.
        bv = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance_position])
        pcm.constraint_region.primitives = [bv]
        pcm.constraint_region.primitive_poses = [Pose(pose.position, Quaternion(0, 0, 0, 1))]

        ocm = OrientationConstraint(header=header, link_name=ee_link)
        ocm.orientation = Quaternion.from_frame(frame)
        ocm.absolute_x_axis_tolerance = tolerance_angle
        ocm.absolute_y_axis_tolerance = tolerance_angle
        ocm.absolute_z_axis_tolerance = tolerance_angle

        # TODO: possibility to hand over more goal constraints
        goal_constraints = [Constraints(position_constraints=[pcm], orientation_constraints=[ocm])]

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

        self.GET_MOTION_PLAN(self, request, callback, errback)

    @validated_response
    def motion_plan_goal_joint_positions(self, joint_positions_goal, joint_names_goal, tolerances,
                                         base_link, group, joint_names, joint_positions,
                                         path_constraints=None, trajectory_constraints=None,
                                         planner_id='', num_planning_attempts=8,
                                         allowed_planning_time=2., max_velocity_scaling_factor=1.,
                                         max_acceleration_scaling_factor=1.):
        kwargs = {}
        kwargs['joint_positions_goal'] = joint_positions_goal
        kwargs['joint_names_goal'] = joint_names_goal
        kwargs['tolerances'] = tolerances
        kwargs['base_link'] = base_link
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

        kwargs['errback_name'] = 'errback'

        return await_callback(self.motion_plan_goal_joint_positions_async, **kwargs)

    def motion_plan_goal_joint_positions_async(self, callback, errback,
                                               joint_positions_goal, joint_names_goal, tolerances,
                                               base_link, group, joint_names, joint_positions,
                                               path_constraints=None, trajectory_constraints=None,
                                               planner_id='', num_planning_attempts=8,
                                               allowed_planning_time=2., max_velocity_scaling_factor=1.,
                                               max_acceleration_scaling_factor=1.):
        """
        """
        # http://docs.ros.org/jade/api/moveit_core/html/utils_8cpp_source.html

        header = Header(frame_id=base_link)
        joint_state = JointState(header=header, name=joint_names, position=joint_positions)
        start_state = RobotState(joint_state, MultiDOFJointState(header=header))

        joint_constraints = []
        for position, joint_name, tolerance in zip(joint_positions_goal, joint_names_goal, tolerances):
            jcm = JointConstraint(joint_name, position, tolerance, tolerance)
            joint_constraints.append(jcm)

        # TODO: possibility to hand over more goal constraints
        goal_constraints = [Constraints(joint_constraints=joint_constraints)]

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

        self.GET_MOTION_PLAN(self, request, callback, errback)

    def collision_mesh(self, id_name, root_link, compas_mesh, operation=1):
        """
        """
        co = CollisionObject(header=Header(frame_id=root_link), id=id_name)
        if compas_mesh:
            mesh = Mesh.from_mesh(compas_mesh)
            co.meshes = [mesh]
            co.mesh_poses = [Pose()]
        if operation:
            co.operation = CollisionObject.ADD
        else:
            co.operation = CollisionObject.REMOVE

        topic = Topic(self, '/collision_object', 'moveit_msgs/CollisionObject')
        topic.publish(co.msg)

    def attached_collision_mesh(self, id_name, ee_link, compas_mesh, operation=1, touch_links=[]):
        """
        """
        co = CollisionObject(header=Header(frame_id=ee_link), id=id_name)
        if compas_mesh:
            mesh = Mesh.from_mesh(compas_mesh)
            co.meshes = [mesh]
            co.mesh_poses = [Pose()]
        if operation:
            co.operation = CollisionObject.ADD
        else:
            co.operation = CollisionObject.REMOVE

        aco = AttachedCollisionObject()
        aco.link_name = ee_link
        # The set of links that the attached objects are allowed to touch by default.
        aco.touch_links = touch_links
        aco.object = co

        topic = Topic(self, '/attached_collision_object', 'moveit_msgs/AttachedCollisionObject')
        topic.publish(aco.msg)

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

        action_client = ActionClient(self, '/follow_joint_trajectory',
                                     'control_msgs/FollowJointTrajectoryAction', timeout)
        goal = Goal(action_client, Message(goal.msg))

        goal.on('result', lambda result: handle_result(result, action_client))
        goal.on('feedback', lambda feedback: print(feedback))
        goal.on('timeout', lambda: print('TIMEOUT'))
        action_client.on('timeout', lambda: print('CLIENT TIMEOUT'))
        goal.send(60000)

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

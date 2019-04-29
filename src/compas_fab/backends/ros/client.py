from __future__ import print_function

from compas.utilities import await_callback
from roslibpy import Message
from roslibpy import Ros
from roslibpy import Service
from roslibpy import ServiceRequest
from roslibpy.actionlib import ActionClient
from roslibpy.actionlib import Goal

from compas_fab.backends.ros.exceptions import RosError
from compas_fab.backends.ros.messages import FollowJointTrajectoryGoal
from compas_fab.backends.ros.messages import FollowJointTrajectoryResult
from compas_fab.backends.ros.messages import GetPlanningSceneRequest
from compas_fab.backends.ros.messages import GetPlanningSceneResponse
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointTrajectory
from compas_fab.backends.ros.messages import JointTrajectoryPoint
from compas_fab.backends.ros.messages import PlanningSceneComponents
from compas_fab.backends.ros.messages import Time
from compas_fab.backends.ros.planner_backend_moveit import MoveItPlanner
from compas_fab.backends.tasks import CancellableTask

__all__ = [
    'RosClient',
]

PLANNER_BACKENDS = {
    'moveit': MoveItPlanner
}


class CancellableRosAction(CancellableTask):
    def __init__(self, goal):
        self.goal = goal

    def cancel(self):
        """Attempt to cancel the task.

        If the task is currently being executed and cannot be cancelled
        then the method will return ``False``, otherwise the call will
        be cancelled and the method will return ``True``.
        """
        self.goal.cancel()


class RosClient(Ros):
    """Interface to use ROS as backend via the **rosbridge**.

    The connection is managed by ``roslibpy``.

    :class:`.RosClient` is a context manager type, so it's best
    used in combination with the ``with`` statement to ensure
    resource deallocation.

    Parameters
    ----------
    host : :obj:`str`
        ROS bridge host. Defaults to ``localhost``.
    port : :obj:`int`
        Port of the ROS Bridge. Defaults to ``9090``.
    is_secure : :obj:`bool`
        ``True`` to indicate it should use a secure web socket, otherwise ``False``.
    planner_backend: str
        Name of the planner backend plugin to use. The plugin must be a sub-class of
        :class:`PlannerBackend`. Defaults to :class:`moveit`.

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

    def __init__(self, host='localhost', port=9090, is_secure=False, planner_backend='moveit'):
        super(RosClient, self).__init__(host, port, is_secure)

        # Dynamically mixin the planner plugin into this class
        planner_backend_type = PLANNER_BACKENDS[planner_backend]
        self.__class__ = type('RosClient_' + planner_backend_type.__name__, (planner_backend_type, RosClient), {})

    def __enter__(self):
        self.run()
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()

    def inverse_kinematics(self, frame, base_link, group,
                           joint_names, joint_positions, avoid_collisions=True,
                           constraints=None, attempts=8, 
                           attached_collision_meshes=None):
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

    def forward_kinematics(self, joint_positions, base_link, group, joint_names, ee_link):
        kwargs = {}
        kwargs['joint_positions'] = joint_positions
        kwargs['base_link'] = base_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['ee_link'] = ee_link

        kwargs['errback_name'] = 'errback'

        return await_callback(self.forward_kinematics_async, **kwargs)

    def plan_cartesian_motion(self, frames, base_link,
                              ee_link, group, joint_names, joint_types,
                              start_configuration, max_step, avoid_collisions,
                              path_constraints, attached_collision_meshes):
        kwargs = {}
        kwargs['frames'] = frames
        kwargs['base_link'] = base_link
        kwargs['ee_link'] = ee_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_types'] = joint_types
        kwargs['start_configuration'] = start_configuration
        kwargs['max_step'] = max_step
        kwargs['avoid_collisions'] = avoid_collisions
        kwargs['path_constraints'] = path_constraints
        kwargs['attached_collision_meshes'] = attached_collision_meshes

        kwargs['errback_name'] = 'errback'

        return await_callback(self.plan_cartesian_motion_async, **kwargs)

    def plan_motion(self, goal_constraints, base_link, ee_link, group,
                    joint_names, joint_types, start_configuration, path_constraints=None,
                    trajectory_constraints=None, planner_id='',
                    num_planning_attempts=8, allowed_planning_time=2.,
                    max_velocity_scaling_factor=1.,
                    max_acceleration_scaling_factor=1.,
                    attached_collision_meshes=None,
                    workspace_parameters=None):

        kwargs = {}
        kwargs['goal_constraints'] = goal_constraints
        kwargs['base_link'] = base_link
        kwargs['ee_link'] = ee_link
        kwargs['group'] = group
        kwargs['joint_names'] = joint_names
        kwargs['joint_types'] = joint_types
        kwargs['start_configuration'] = start_configuration
        kwargs['path_constraints'] = path_constraints
        kwargs['trajectory_constraints'] = trajectory_constraints
        kwargs['planner_id'] = planner_id
        kwargs['num_planning_attempts'] = num_planning_attempts
        kwargs['allowed_planning_time'] = allowed_planning_time
        kwargs['max_velocity_scaling_factor'] = max_velocity_scaling_factor
        kwargs['max_acceleration_scaling_factor'] = max_acceleration_scaling_factor
        kwargs['attached_collision_meshes'] = attached_collision_meshes
        kwargs['workspace_parameters'] = workspace_parameters

        kwargs['errback_name'] = 'errback'

        return await_callback(self.plan_motion_async, **kwargs)

    def inverse_kinematics_async(self, *args, **kwargs):
        raise NotImplementedError('No planner backend assigned')

    def forward_kinematics_async(self, *args, **kwargs):
        raise NotImplementedError('No planner backend assigned')

    def plan_motion_async(self, *args, **kwargs):
        raise NotImplementedError('No planner backend assigned')

    def plan_cartesian_motion_async(self, *args, **kwargs):
        raise NotImplementedError('No planner backend assigned')

    # ==========================================================================
    # collision objects
    # ==========================================================================

    def add_collision_mesh(self, collision_mesh):
        raise NotImplementedError('No planner backend assigned')

    def remove_collision_mesh(self, id):
        raise NotImplementedError('No planner backend assigned')

    def append_collision_mesh(self, collision_mesh):
        raise NotImplementedError('No planner backend assigned')

    def add_attached_collision_mesh(self, attached_collision_mesh):
        raise NotImplementedError('No planner backend assigned')

    def remove_attached_collision_mesh(self, id):
        raise NotImplementedError('No planner backend assigned')

    # ==========================================================================
    # executing
    # ==========================================================================

    def follow_configurations(self, callback, joint_names, configurations, timesteps, timeout=None):

        if len(configurations) != len(timesteps):
            raise ValueError("%d configurations must have %d timesteps, but %d given." % (
                len(configurations), len(timesteps), len(timesteps)))

        if not timeout:
            timeout = timesteps[-1] * 1000 * 2

        points = []
        num_joints = len(configurations[0].values)
        for config, time in zip(configurations, timesteps):
            pt = JointTrajectoryPoint(positions=config.values, velocities=[
                                      0]*num_joints, time_from_start=Time(secs=(time)))
            points.append(pt)

        joint_trajectory = JointTrajectory(
            Header(), joint_names, points)  # specify header necessary?
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

        trajectory_goal = FollowJointTrajectoryGoal(
            trajectory=joint_trajectory)

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
            goal.on('result', lambda result: handle_result(
                result, action_client))

        if feedback_callback:
            goal.on('feedback', feedback_callback)

        if errback:
            goal.on('timeout', lambda: handle_failure(
                RosError("Action Goal timeout", -1)))
            action_client.on('timeout', lambda: handle_failure(
                RosError("Actionlib client timeout", -1)))

        goal.send(timeout=timeout)

        return CancellableRosAction(goal)

    def get_planning_scene(self, callback, components):
        """
        """
        reqmsg = GetPlanningSceneRequest(PlanningSceneComponents(components))

        def receive_message(msg):
            response = GetPlanningSceneResponse.from_msg(msg)
            callback(response)

        srv = Service(self, '/get_planning_scene',
                      'moveit_msgs/GetPlanningScene')
        request = ServiceRequest(reqmsg.msg)
        srv.call(request, receive_message, receive_message)

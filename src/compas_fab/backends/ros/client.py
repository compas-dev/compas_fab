from __future__ import print_function

import os

from compas.robots import RobotModel
from roslibpy import Message
from roslibpy import Ros
from roslibpy.actionlib import ActionClient
from roslibpy.actionlib import Goal

from compas_fab.backends.interfaces.client import ClientInterface
from compas_fab.backends.ros.exceptions import RosError
from compas_fab.backends.ros.fileserver_loader import RosFileServerLoader
from compas_fab.backends.ros.messages import ExecuteTrajectoryFeedback
from compas_fab.backends.ros.messages import ExecuteTrajectoryGoal
from compas_fab.backends.ros.messages import ExecuteTrajectoryResult
from compas_fab.backends.ros.messages import FollowJointTrajectoryFeedback
from compas_fab.backends.ros.messages import FollowJointTrajectoryGoal
from compas_fab.backends.ros.messages import FollowJointTrajectoryResult
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import JointTrajectory as RosMsgJointTrajectory
from compas_fab.backends.ros.messages import JointTrajectoryPoint as RosMsgJointTrajectoryPoint
from compas_fab.backends.ros.messages import MoveItErrorCodes
from compas_fab.backends.ros.messages import RobotTrajectory
from compas_fab.backends.ros.messages import Time
from compas_fab.backends.ros.planner import MoveItPlanner
from compas_fab.backends.tasks import CancellableFutureResult
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics

__all__ = [
    'RosClient',
]

PLANNER_BACKENDS = {
    'moveit': MoveItPlanner
}


class CancellableRosActionResult(CancellableFutureResult):
    def __init__(self, goal):
        super(CancellableRosActionResult, self).__init__()
        self.goal = goal

    def cancel(self):
        """Attempt to cancel the action.

        If the task is currently being executed and cannot be cancelled
        then the method will return ``False``, otherwise the call will
        be cancelled and the method will return ``True``.
        """
        if self.done:
            raise Exception('Already completed action cannot be cancelled')

        if self.goal.is_finished:
            return False

        self.goal.cancel()

        # NOTE: Check if we can output more meaning results than just "we tried to cancel"
        return True


class LocalCacheInfo(object):
    def __init__(self, use_local_cache, robot_name=None, local_cache_directory=None):
        self.use_local_cache = use_local_cache
        self.robot_name = robot_name
        self.local_cache_directory = local_cache_directory

    @classmethod
    def from_local_cache_directory(cls, local_cache_directory):
        """Construct local caching options based on the local cache directory.

        Based on the specified local cache directory, determine whether
        caching should be used at all, what the robot name is, and what the
        real caching directory to use is (which differs from the local cache
        directory specified to this function).

        Examples
        --------
        >>> info = LocalCacheInfo.from_local_cache_directory(None)
        >>> info.use_local_cache
        False


        >>> local_directory = os.path.join(os.path.expanduser('~'), 'robot_description', 'robocop')
        >>> info = LocalCacheInfo.from_local_cache_directory(local_directory)
        >>> info.use_local_cache
        True
        >>> info.robot_name
        'robocop'
        """
        if local_cache_directory is None:
            return LocalCacheInfo(use_local_cache=False)

        path_parts = local_cache_directory.rstrip(os.path.sep).split(os.path.sep)
        path_parts, robot_name = path_parts[:-1], path_parts[-1]
        local_cache_directory = os.path.sep.join(path_parts)

        return LocalCacheInfo(use_local_cache=True,
                              robot_name=robot_name,
                              local_cache_directory=local_cache_directory)


class RosClient(Ros, ClientInterface):
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
        :class:`compas_fab.backends.PlannerInterface`. Defaults to ``"moveit"``,
        making use of :class:`compas_fab.backends.MoveItPlanner`.

    Examples
    --------

    >>> from compas_fab.backends import RosClient
    >>> with RosClient() as client:
    ...     print('Connected: %s' % client.is_connected)
    Connected: True

    Note
    ----
    For more examples, check out the :ref:`ROS examples page <ros_examples>`.
    """

    def __init__(self, host='localhost', port=9090, is_secure=False, planner_backend='moveit'):
        super(RosClient, self).__init__(host, port, is_secure)

        planner_backend_type = PLANNER_BACKENDS[planner_backend]
        self.planner = planner_backend_type(self)

    def __enter__(self):
        self.run()

        return self

    def __exit__(self, *args):
        self.close()

    def load_robot(self, load_geometry=False, urdf_param_name='/robot_description', srdf_param_name='/robot_description_semantic', precision=None, local_cache_directory=None):
        """Load an entire robot instance -including model and semantics- directly from ROS.

        Parameters
        ----------
        load_geometry : bool, optional
            ``True`` to load the robot's geometry, otherwise ``False`` to load only the model and semantics.
        urdf_param_name : str, optional
            Parameter name where the URDF is defined. If not defined, it will default to ``/robot_description``.
        srdf_param_name : str, optional
            Parameter name where the SRDF is defined. If not defined, it will default to ``/robot_description_semantic``.
        precision : float
            Defines precision for importing/loading meshes. Defaults to ``compas.PRECISION``.
        local_cache_directory : str, optional
            Directory where the robot description (URDF, SRDF and meshes) are stored.
            This differs from the directory taken as parameter by the :class:`RosFileServerLoader`
            in that it points directly to the specific robot package, not to a global workspace storage
            for all robots. If not assigned, the robot will not be cached locally.

        Examples
        --------

        >>> from compas_fab.backends import RosClient
        >>> with RosClient() as client:
        ...     robot = client.load_robot()
        ...     print(robot.name)
        ur5
        """
        robot_name = None

        cache_info = LocalCacheInfo.from_local_cache_directory(local_cache_directory)
        use_local_cache = cache_info.use_local_cache
        robot_name = cache_info.robot_name
        local_cache_directory = cache_info.local_cache_directory

        loader = RosFileServerLoader(self, use_local_cache, local_cache_directory, precision)

        if robot_name:
            loader.robot_name = robot_name

        urdf = loader.load_urdf(urdf_param_name)
        srdf = loader.load_srdf(srdf_param_name)

        model = RobotModel.from_urdf_string(urdf)
        semantics = RobotSemantics.from_srdf_string(srdf, model)

        if load_geometry:
            model.load_geometry(loader)

        return Robot(model, semantics=semantics, client=self)

    # ==========================================================================
    # executing
    # ==========================================================================

    def get_configuration(self):
        pass

    def follow_configurations(self, callback, joint_names, configurations, timesteps, timeout=60000):

        if len(configurations) != len(timesteps):
            raise ValueError("%d configurations must have %d timesteps, but %d given." % (
                len(configurations), len(timesteps), len(timesteps)))

        if not timeout:
            timeout = timesteps[-1] * 1000 * 2

        points = []
        num_joints = len(configurations[0].values)
        for config, time in zip(configurations, timesteps):
            pt = RosMsgJointTrajectoryPoint(positions=config.values, velocities=[
                                      0]*num_joints, time_from_start=Time(secs=(time)))
            points.append(pt)

        joint_trajectory = RosMsgJointTrajectory(
            Header(), joint_names, points)  # specify header necessary?
        return self.follow_joint_trajectory(joint_trajectory=joint_trajectory, callback=callback, timeout=timeout)

    def _convert_to_ros_trajectory(self, joint_trajectory):
        """Converts a ``compas_fab.robots.JointTrajectory`` into a ROS Msg joint trajectory."""

        # For backwards-compatibility, accept ROS Msg directly as well and simply do not modify
        if isinstance(joint_trajectory, RosMsgJointTrajectory):
            return joint_trajectory

        trajectory = RosMsgJointTrajectory()
        trajectory.joint_names = joint_trajectory.joint_names

        for point in joint_trajectory.points:
            ros_point = RosMsgJointTrajectoryPoint(
                positions=point.positions,
                velocities=point.velocities,
                accelerations=point.accelerations,
                effort=point.effort,
                time_from_start=Time(
                    point.time_from_start.secs, point.time_from_start.nsecs),
            )
            trajectory.points.append(ros_point)

        return trajectory

    def follow_joint_trajectory(self, joint_trajectory, action_name='/joint_trajectory_action', callback=None, errback=None, feedback_callback=None, timeout=60000):
        """Follow the joint trajectory as computed by MoveIt planner.

        Parameters
        ----------
        joint_trajectory : :class:`compas_fab.robots.JointTrajectory`
            Instance of joint trajectory. Note: for backwards compatibility, this supports a ROS Msg being passed as well.
        action_name : string
            ROS action name, defaults to ``/joint_trajectory_action`` but some drivers need ``/follow_joint_trajectory``.
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

        joint_trajectory = self._convert_to_ros_trajectory(joint_trajectory)
        trajectory_goal = FollowJointTrajectoryGoal(trajectory=joint_trajectory)

        action_client = ActionClient(self, action_name, 'control_msgs/FollowJointTrajectoryAction')
        goal = Goal(action_client, Message(trajectory_goal.msg))
        action_result = CancellableRosActionResult(goal)

        def handle_result(msg):
            result = FollowJointTrajectoryResult.from_msg(msg)
            if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                ros_error = RosError(
                    'Follow trajectory failed={}'.format(result.error_string),
                    int(result.error_code))
                action_result._set_error_result(ros_error)
                if errback:
                    errback(ros_error)
            else:
                action_result._set_result(result)
                if callback:
                    callback(result)

        def handle_feedback(msg):
            feedback = FollowJointTrajectoryFeedback.from_msg(msg)
            feedback_callback(feedback)

        def handle_failure(error):
            errback(error)

        goal.on('result', handle_result)

        if feedback_callback:
            goal.on('feedback', handle_feedback)

        if errback:
            goal.on('timeout', lambda: handle_failure(
                RosError('Action Goal timeout', -1)))

        goal.send(timeout=timeout)

        return action_result

    def execute_joint_trajectory(self, joint_trajectory, action_name='/execute_trajectory', callback=None, errback=None, feedback_callback=None, timeout=60000):
        """Execute a joint trajectory via the MoveIt infrastructure.

        Note
        ----
        This method does not support Multi-DOF Joint Trajectories.

        Parameters
        ----------
        joint_trajectory : :class:`compas_fab.robots.JointTrajectory`
            Instance of joint trajectory.
        callback : callable
            Function to be invoked when the goal is completed, requires
            one positional parameter ``result``.
        action_name : string
            ROS action name, defaults to ``/execute_trajectory``.
        errback : callable
            Function to be invoked in case of error or timeout, requires
            one position parameter ``exception``.
        feedback_callback : callable
            Function to be invoked during execution to provide feedback.
        timeout : int
            Timeout for goal completion in milliseconds.

        Returns
        -------
        :class:`CancellableFutureResult`
            An instance of a cancellable future result.
        """

        joint_trajectory = self._convert_to_ros_trajectory(joint_trajectory)
        trajectory = RobotTrajectory(joint_trajectory=joint_trajectory)
        trajectory_goal = ExecuteTrajectoryGoal(trajectory=trajectory)

        action_client = ActionClient(self, action_name, 'moveit_msgs/ExecuteTrajectoryAction')
        goal = Goal(action_client, Message(trajectory_goal.msg))
        action_result = CancellableRosActionResult(goal)

        def handle_result(msg):
            result = ExecuteTrajectoryResult.from_msg(msg)
            if result.error_code != MoveItErrorCodes.SUCCESS:
                ros_error = RosError('Execute trajectory failed. Message={}'.format(result.error_code.human_readable), int(result.error_code))
                action_result._set_error_result(ros_error)
                if errback:
                    errback(ros_error)
            else:
                action_result._set_result(result)
                if callback:
                    callback(result)

        def handle_feedback(msg):
            feedback = ExecuteTrajectoryFeedback.from_msg(msg)
            feedback_callback(feedback)

        def handle_failure(error):
            errback(error)

        goal.on('result', handle_result)

        if feedback_callback:
            goal.on('feedback', handle_feedback)

        if errback:
            goal.on('timeout', lambda: handle_failure(RosError('Action Goal timeout', -1)))

        goal.send(timeout=timeout)

        return action_result

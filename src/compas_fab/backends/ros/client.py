import os
from typing import Optional

from compas_robots import RobotModel
from roslibpy import ActionClient as Ros2ActionClient
from roslibpy import Message
from roslibpy import Param
from roslibpy import Ros
from roslibpy import Service
from roslibpy import ServiceRequest
from roslibpy.ros1.actionlib import ActionClient
from roslibpy.ros1.actionlib import Goal

from compas_fab.backends.interfaces.client import ClientInterface
from compas_fab.backends.ros.exceptions import RosError
from compas_fab.backends.ros.fileserver_loader import RosFileServerLoader
from compas_fab.backends.ros.http_fileserver_loader import HttpFileServerLoader
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
from compas_fab.backends.ros.messages import RosDistro
from compas_fab.backends.ros.messages import Time
from compas_fab.backends.tasks import CancellableFutureResult
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotSemantics

__all__ = [
    "RosClient",
]


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
            raise Exception("Already completed action cannot be cancelled")

        if self.goal.is_finished:
            return False

        self.goal.cancel()

        # NOTE: Check if we can output more meaning results than just "we tried to cancel"
        return True


class _Ros2GoalHandle(object):
    """Thin Goal-like adapter over `roslibpy.ActionClient` (ROS 2 actions).

    `roslibpy.ros1.actionlib.Goal` is a topic-driven state machine; the ROS 2
    `ActionClient` is a single ``send_goal`` call with result/feedback/error
    callbacks and a separately-tracked ``goal_id``. To keep the public
    `CancellableRosActionResult` interface unchanged, this wrapper exposes
    the few attributes that wrapper reads (`is_finished`, `cancel`) and lets
    the existing `follow_joint_trajectory` / `execute_joint_trajectory`
    plumbing stay distro-agnostic.
    """

    def __init__(self, action_client):
        self.action_client = action_client
        self.goal_id = None
        self.is_finished = False

    def send(self, goal_dict, on_result, on_feedback=None, on_error=None):
        def _resultback(result):
            self.is_finished = True
            on_result(result)

        def _errback(error):
            self.is_finished = True
            if on_error:
                on_error(error)

        self.goal_id = self.action_client.send_goal(
            goal_dict,
            _resultback,
            on_feedback if on_feedback is not None else (lambda _msg: None),
            _errback,
        )

    def cancel(self):
        if self.goal_id is not None:
            self.action_client.cancel_goal(self.goal_id)


class LocalCacheInfo:
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


        >>> local_directory = os.path.join(os.path.expanduser("~"), "robot_description", "robocop")
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

        return LocalCacheInfo(use_local_cache=True, robot_name=robot_name, local_cache_directory=local_cache_directory)


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

    Examples
    --------
    >>> with RosClient() as client:  # doctest: +SKIP
    ...     print("Connected:", client.is_connected)
    Connected: True
    """

    def __init__(self, host="localhost", port=9090, is_secure=False):
        # `Ros.__init__` is called via super, but `ClientInterface.__init__`
        # is bypassed because `Ros` is the first base in the MRO and does
        # not call `super().__init__()`. We initialise the ClientInterface
        # attributes manually so `self.robot_cell` etc. work before any
        # `load_robot_cell` call.
        super(RosClient, self).__init__(host, port, is_secure)
        ClientInterface.__init__(self)
        self.host = host
        self.port = port
        self.is_secure = is_secure
        self._ros_distro = None

    def __enter__(self):
        self.run()

        return self

    def __exit__(self, *args):
        self.close()

    @property
    def ros_distro(self) -> RosDistro:
        """Retrieves the ROS version to which the client is connected (e.g. ``noetic``, ``jazzy``).

        Modern ``rosapi`` exposes ``/rosapi/get_ros_version`` for both ROS 1 and
        ROS 2. Older ROS 1 deployments also expose a global ``/rosdistro``
        parameter, which remains as a fallback for compatibility.
        """
        if not self._ros_distro:
            value = self._get_ros_distro_from_rosapi()
            if not value:
                value = self._get_ros_distro_from_param()
            if value:
                self._ros_distro = RosDistro(value)
            else:
                # Last-resort default for ROS 2 rosbridge instances without the
                # version service and without the ROS 1 global `/rosdistro` param.
                self._ros_distro = RosDistro.JAZZY

        return self._ros_distro

    def _get_ros_distro_from_rosapi(self) -> str:
        try:
            response = Service(self, "/rosapi/get_ros_version", "rosapi_msgs/GetROSVersion").call(ServiceRequest(), timeout=1)
            value = response.get("distro", "")
            return value.strip() if value else ""
        except Exception:
            return ""

    def _get_ros_distro_from_param(self) -> str:
        try:
            value = Param(self, "/rosdistro").get(timeout=1)
            return value.strip() if value else ""
        except Exception:
            return ""

    def load_robot_cell(
        self,
        load_geometry: bool = False,
        urdf_param_name: str = "/robot_description",
        srdf_param_name: str = "/robot_description_semantic",
        precision: int = None,
        local_cache_directory: Optional[str] = None,
        http_file_server_base_url: Optional[str] = None,
    ):
        """Load the robot cell (including model and semantics) from ROS.
        The robot celL is set in `client.robot_cell` and returned.

        Parameters
        ----------
        load_geometry : bool, optional
            ``True`` to load the robot's geometry, otherwise ``False`` to load only the model and semantics.
        urdf_param_name : str, optional
            Parameter name where the URDF is defined. If not defined, it will default to ``/robot_description``.
        srdf_param_name : str, optional
            Parameter name where the SRDF is defined. If not defined, it will default to ``/robot_description_semantic``.
        precision : int
            Defines precision for importing/loading meshes. Defaults to ``compas.tolerance.TOL.precision``.
        local_cache_directory : str, optional
            Directory where the robot description (URDF, SRDF and meshes) are stored.
            This differs from the directory taken as parameter by the :class:`RosFileServerLoader`
            in that it points directly to the specific robot package, not to a global workspace storage
            for all robots. If not assigned, the robot will not be cached locally.
        http_file_server_base_url : str, optional
            Base URL of the HTTP file server used by ROS 2 to resolve ``package://`` mesh URLs.
            Defaults to ``http://<rosbridge-host>:9190``.

        Examples
        --------
        >>> with RosClient() as client:  # doctest: +SKIP
        ...     robot_cell = client.load_robot_cell()
        ...     print(robot_cell.robot_model.name)
        ur5_robot

        """
        robot_name = None

        cache_info = LocalCacheInfo.from_local_cache_directory(local_cache_directory)
        use_local_cache = cache_info.use_local_cache
        robot_name = cache_info.robot_name
        local_cache_directory = cache_info.local_cache_directory

        loader = self._get_robot_cell_loader(use_local_cache, local_cache_directory, http_file_server_base_url)

        if robot_name:
            loader.robot_name = robot_name

        urdf = loader.load_urdf(urdf_param_name)
        srdf = loader.load_srdf(srdf_param_name)

        model = RobotModel.from_urdf_string(urdf)
        semantics = RobotSemantics.from_srdf_string(srdf, model)

        if load_geometry:
            model.load_geometry(loader, precision=precision)

        self._robot_cell = RobotCell(robot_model=model, robot_semantics=semantics)
        self._robot_cell_state = self._robot_cell.default_cell_state()

        return self._robot_cell

    def _get_robot_cell_loader(self, use_local_cache=False, local_cache_directory=None, http_file_server_base_url=None):
        if self.ros_distro.is_ros2:
            return HttpFileServerLoader(
                http_file_server_base_url or self._default_http_file_server_base_url(),
                ros=self,
                local_cache=use_local_cache,
                local_cache_directory=local_cache_directory,
            )

        return RosFileServerLoader(self, use_local_cache, local_cache_directory)

    def _default_http_file_server_base_url(self):
        return "http://{}:9190".format(self.host)

    # ==========================================================================
    # executing
    # ==========================================================================

    def get_configuration(self):
        pass

    def follow_configurations(self, callback, joint_names, configurations, timesteps, timeout=60000):
        if len(configurations) != len(timesteps):
            raise ValueError("%d configurations must have %d timesteps, but %d given." % (len(configurations), len(timesteps), len(timesteps)))

        if not timeout:
            timeout = timesteps[-1] * 1000 * 2

        points = []
        num_joints = len(configurations[0].joint_values)
        for config, time in zip(configurations, timesteps):
            pt = RosMsgJointTrajectoryPoint(positions=config.joint_values, velocities=[0] * num_joints, time_from_start=Time(secs=(time)))
            points.append(pt)

        joint_trajectory = RosMsgJointTrajectory(Header(), joint_names, points)  # specify header necessary?
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
                time_from_start=Time(point.time_from_start.secs, point.time_from_start.nsecs),
            )
            trajectory.points.append(ros_point)

        return trajectory

    def follow_joint_trajectory(
        self,
        joint_trajectory,
        action_name="/joint_trajectory_action",
        callback=None,
        errback=None,
        feedback_callback=None,
        timeout=60000,
    ):
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

        def handle_result(msg):
            result = FollowJointTrajectoryResult.from_msg(msg)
            if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                ros_error = RosError("Follow trajectory failed={}".format(result.error_string), int(result.error_code))
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

        if self.ros_distro.is_ros2:
            action_client = Ros2ActionClient(self, action_name, "control_msgs/action/FollowJointTrajectory")
            goal = _Ros2GoalHandle(action_client)
            action_result = CancellableRosActionResult(goal)
            goal.send(
                trajectory_goal.msg,
                on_result=handle_result,
                on_feedback=handle_feedback if feedback_callback else None,
                on_error=errback,
            )
            return action_result

        action_client = ActionClient(self, action_name, "control_msgs/FollowJointTrajectoryAction")
        goal = Goal(action_client, Message(trajectory_goal.msg))
        action_result = CancellableRosActionResult(goal)

        def handle_failure(error):
            errback(error)

        goal.on("result", handle_result)

        if feedback_callback:
            goal.on("feedback", handle_feedback)

        if errback:
            goal.on("timeout", lambda: handle_failure(RosError("Action Goal timeout", -1)))

        goal.send(timeout=timeout)

        return action_result

    def execute_joint_trajectory(
        self,
        joint_trajectory,
        action_name="/execute_trajectory",
        callback=None,
        errback=None,
        feedback_callback=None,
        timeout=60000,
    ):
        """Execute a joint trajectory via the MoveIt infrastructure.

        Notes
        -----
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

        def handle_result(msg):
            result = ExecuteTrajectoryResult.from_msg(msg)
            if result.error_code != MoveItErrorCodes.SUCCESS:
                ros_error = RosError(
                    "Execute trajectory failed. Message={}".format(result.error_code.human_readable),
                    int(result.error_code),
                )
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

        if self.ros_distro.is_ros2:
            action_client = Ros2ActionClient(self, action_name, "moveit_msgs/action/ExecuteTrajectory")
            goal = _Ros2GoalHandle(action_client)
            action_result = CancellableRosActionResult(goal)
            goal.send(
                trajectory_goal.msg,
                on_result=handle_result,
                on_feedback=handle_feedback if feedback_callback else None,
                on_error=errback,
            )
            return action_result

        action_client = ActionClient(self, action_name, "moveit_msgs/ExecuteTrajectoryAction")
        goal = Goal(action_client, Message(trajectory_goal.msg))
        action_result = CancellableRosActionResult(goal)

        def handle_failure(error):
            errback(error)

        goal.on("result", handle_result)

        if feedback_callback:
            goal.on("feedback", handle_feedback)

        if errback:
            goal.on("timeout", lambda: handle_failure(RosError("Action Goal timeout", -1)))

        goal.send(timeout=timeout)

        return action_result

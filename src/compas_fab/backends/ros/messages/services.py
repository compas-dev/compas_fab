from __future__ import absolute_import

from .geometry_msgs import PoseStamped
from .moveit_msgs import Constraints
from .moveit_msgs import MoveItErrorCodes
from .moveit_msgs import PlannerParams
from .moveit_msgs import PlanningScene
from .moveit_msgs import PlanningSceneComponents
from .moveit_msgs import PositionIKRequest
from .moveit_msgs import RobotState
from .moveit_msgs import RobotTrajectory
from .moveit_msgs import TrajectoryConstraints
from .moveit_msgs import WorkspaceParameters
from .std_msgs import Header
from .std_msgs import ROSmsg


class GetPositionIKRequest(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/srv/GetPositionIK.html

    Examples
    --------
    >>> import roslibpy
    >>> base_link = 'base_link'  # robot.get_base_link_name()
    >>> planning_group = 'manipulator'  # robot.main_planning_group
    >>> pose = Pose([0.420, -0.025, 0.459], [1, 0, 0], [0, 1, 0])
    >>> joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                       'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                       'wrist_3_joint']
    >>> joint_positions = [3.39, -1.47, -2.05, 0.38, -4.96, -6.28]
    >>> header = Header(frame_id='base_link')
    >>> pose_stamped = PoseStamped(header, pose)
    >>> joint_state = JointState(name=joint_names, position=joint_positions,
                                 header=header)
    >>> multi_dof_joint_state = MultiDOFJointState(header=header)
    >>> start_state = RobotState(joint_state, multi_dof_joint_state)
    >>> ik_request = PositionIKRequest(group_name=planning_group,
                                       robot_state=start_state,
                                       pose_stamped=pose_stamped,
                                       avoid_collisions=True)
    >>> reqmsg = GetPositionIKRequest(ik_request)
    >>> srv = roslibpy.Service(ros_client, '/compute_ik', 'GetPositionIK')
    >>> request = roslibpy.ServiceRequest(reqmsg.msg)
    >>> srv.call(request, GetPositionIKResponse.from_msg, GetPositionIKResponse.from_msg)
    """
    def __init__(self, ik_request=None):
        self.ik_request = ik_request or PositionIKRequest()


class GetPositionIKResponse(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/srv/GetPositionIK.html
    """

    def __init__(self, solution=None, error_code=None):
        self.solution = solution or RobotState()  # moveit_msgs/RobotState
        self.error_code = error_code or MoveItErrorCodes()  # moveit_msgs/MoveItErrorCodes

    @classmethod
    def from_msg(cls, msg):
        solution = RobotState.from_msg(msg['solution'])
        error_code = MoveItErrorCodes.from_msg(msg['error_code'])
        return cls(solution, error_code)


class GetPositionFKRequest(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/srv/GetPositionFK.html
    """

    def __init__(self, header=None, fk_link_names=None, robot_state=None):
        self.header = header or Header()
        self.fk_link_names = fk_link_names or []
        self.robot_state = robot_state or RobotState()


class GetPositionFKResponse(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/srv/GetPositionFK.html
    """

    def __init__(self, pose_stamped=None, fk_link_names=None, error_code=None):
        self.pose_stamped = pose_stamped or []  # PoseStamped[]
        self.fk_link_names = fk_link_names or []
        self.error_code = error_code or MoveItErrorCodes()  # moveit_msgs/MoveItErrorCodes

    @classmethod
    def from_msg(cls, msg):
        pose_stamped = [PoseStamped.from_msg(d) for d in msg['pose_stamped']]
        fk_link_names = msg['fk_link_names']
        error_code = MoveItErrorCodes.from_msg(msg['error_code'])
        return cls(pose_stamped, fk_link_names, error_code)


class GetCartesianPathRequest(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/srv/GetCartesianPath.html

    Examples
    --------
    >>> import roslibpy
    >>> base_link = 'base_link' # robot.get_base_link_name()
    >>> ee_link = 'ee_link' # robot.get_end_effector_link_name()
    >>> main_planning_group = 'manipulator' # robot.main_planning_group
    >>> joint_names = ['j0', 'j1', 'j2', 'j3', 'j4', 'j5']
    >>> position = [0, 0, 0, 0, 0, 0] # robot.get_configurable_joint_names()
    >>> header = Header(frame_id=base_link)
    >>> joint_state = JointState(header=header, name=joint_names, position=position) # or: robot.get_joint_state()
    >>> multi_dof_joint_state = MultiDOFJointState(header=header)
    >>> start_state = RobotState(joint_state=joint_state, multi_dof_joint_state=multi_dof_joint_state)
    >>> start_pose = Pose([0.1068, -0.1818, 0.5930], [1., 0., 0.], [-0., 0., 1.])
    >>> end_pose = Pose([0.1041, -0.2946, 0.1843], [1., 0., 0.], [0., 1., 0.])
    >>> waypoints = [start_pose, end_pose]
    >>> reqmsg = GetCartesianPathRequest(header=header,
                                         start_state=start_state,
                                         group_name=main_planning_group,
                                         link_name=ee_link,
                                         waypoints=waypoints,
                                         max_step=10,
                                         avoid_collisions=True)
    >>> srv = roslibpy.Service(ros_client, '/compute_cartesian_path', 'GetCartesianPath')
    >>> request = roslibpy.ServiceRequest(reqmsg.msg)
    >>> srv.call(request, GetCartesianPathResponse.from_msg, GetCartesianPathResponse.from_msg)
    """

    def __init__(self, header=None, start_state=None, group_name='',
                 link_name='', waypoints=None, max_step=10., jump_threshold=0.,
                 avoid_collisions=True, path_constraints=None):
        self.header = header or Header()
        self.start_state = start_state or RobotState()  # moveit_msgs/RobotState
        self.group_name = group_name
        self.link_name = link_name  # ee_link
        self.waypoints = waypoints if waypoints else []  # geometry_msgs/Pose[]
        self.max_step = float(max_step)
        self.jump_threshold = jump_threshold
        self.avoid_collisions = avoid_collisions
        self.path_constraints = path_constraints or Constraints()  # moveit_msgs/Constraints


class GetCartesianPathResponse(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/srv/GetCartesianPath.html
    """

    def __init__(self, start_state=None, solution=None,
                 fraction=0., error_code=None):
        self.start_state = start_state or RobotState()  # moveit_msgs/RobotState
        self.solution = solution or RobotTrajectory()  # moveit_msgs/RobotTrajectory
        self.fraction = fraction
        self.error_code = error_code or MoveItErrorCodes()  # moveit_msgs/MoveItErrorCodes

    @classmethod
    def from_msg(cls, msg):
        start_state = RobotState.from_msg(msg['start_state'])
        solution = RobotTrajectory.from_msg(msg['solution'])
        error_code = MoveItErrorCodes.from_msg(msg['error_code'])
        return cls(start_state, solution, msg['fraction'], error_code)


class SetPlannerParamsRequest(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/srv/SetPlannerParams.html
    """

    def __init__(self, planner_config='', group='', params=None, replace=True):
        self.planner_config = planner_config
        self.group = group
        self.params = params or PlannerParams()
        self.replace = replace


class MotionPlanRequest(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/MotionPlanRequest.html
    """
    def __init__(self, workspace_parameters=None, start_state=None,
                 goal_constraints=None, path_constraints=None,
                 trajectory_constraints=None, planner_id='',
                 group_name='', num_planning_attempts=8,
                 allowed_planning_time=2., max_velocity_scaling_factor=1.,
                 max_acceleration_scaling_factor=1.):
        self.workspace_parameters = workspace_parameters or WorkspaceParameters()  # moveit_msgs/WorkspaceParameters
        self.start_state = start_state or RobotState()  # moveit_msgs/RobotState
        self.goal_constraints = goal_constraints or []  # moveit_msgs/Constraints[]
        self.path_constraints = path_constraints or Constraints()  # moveit_msgs/Constraints
        self.trajectory_constraints = trajectory_constraints or TrajectoryConstraints()  # moveit_msgs/TrajectoryConstraints
        self.planner_id = planner_id  # string
        self.group_name = group_name  # string
        self.num_planning_attempts = int(num_planning_attempts)  # int32
        self.allowed_planning_time = float(allowed_planning_time)  # float64
        self.max_velocity_scaling_factor = float(max_velocity_scaling_factor)  # float64
        self.max_acceleration_scaling_factor = float(max_acceleration_scaling_factor)  # float64

    @property
    def msg(self):
        msg = super(MotionPlanRequest, self).msg
        return {"motion_plan_request": msg}


class MotionPlanResponse(ROSmsg):
    """http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/MotionPlanResponse.html
    """

    def __init__(self, trajectory_start=None, group_name=None, trajectory=None,
                 planning_time=None, error_code=None):

        self.trajectory_start = trajectory_start or RobotState()
        self.group_name = group_name or ''
        self.trajectory = trajectory or RobotTrajectory()
        self.planning_time = planning_time or 3.
        self.error_code = error_code or MoveItErrorCodes()

    @classmethod
    def from_msg(cls, msg):
        msg = msg["motion_plan_response"]
        trajectory_start = RobotState.from_msg(msg['trajectory_start'])
        trajectory = RobotTrajectory.from_msg(msg['trajectory'])
        error_code = MoveItErrorCodes.from_msg(msg['error_code'])
        return cls(trajectory_start, msg['group_name'], trajectory, msg['planning_time'], error_code)


class GetPlanningSceneRequest(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/srv/GetPlanningScene.html
    """
    def __init__(self, components=None):
        self.components = components or PlanningSceneComponents()


class GetPlanningSceneResponse(ROSmsg):
    """http://docs.ros.org/melodic/api/moveit_msgs/html/srv/GetPlanningScene.html
    """
    def __init__(self, scene=None):
        self.scene = scene or PlanningScene()

    @classmethod
    def from_msg(cls, msg):
        return PlanningScene.from_msg(msg['scene'])

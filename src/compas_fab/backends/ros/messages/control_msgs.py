from .std_msgs import ROSmsg
from .std_msgs import Header
from .std_msgs import Time
from .actionlib_msgs import GoalID
from .actionlib_msgs import GoalStatus
from .trajectory_msgs import JointTrajectoryPoint
from .trajectory_msgs import JointTrajectory


class JointTolerance(ROSmsg):
    """http://docs.ros.org/api/control_msgs/html/msg/JointTolerance.html
    """
    def __init__(self, name="", position=0., velocity=0., acceleration=0.):
        self.name = name
        self.position = position # in radians or meters (for a revolute or prismatic joint, respectively)
        self.velocity = velocity  # in rad/sec or m/sec
        self.acceleration = acceleration # in rad/sec^2 or m/sec^2


class FollowJointTrajectoryGoal(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryGoal.html
    """

    def __init__(self, trajectory=None, path_tolerance=None, 
                 goal_tolerance=None, goal_time_tolerance=None):
        self.trajectory = trajectory if trajectory else JointTrajectory() #trajectory_msgs/JointTrajectory 
        self.path_tolerance = path_tolerance if path_tolerance else []#control_msgs/JointTolerance[] 
        self.goal_tolerance = goal_tolerance if goal_tolerance else [] #control_msgs/JointTolerance[] 
        self.goal_time_tolerance = goal_time_tolerance if goal_time_tolerance else Time(secs=1.)


class FollowJointTrajectoryActionGoal(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryActionGoal.html
    """

    def __init__(self, header=None, goal_id=None, goal=None):
        self.header = header if header else Header()
        self.goal_id = goal_id if goal_id else GoalID() #actionlib_msgs/GoalID goal_id
        self.goal = goal if goal else FollowJointTrajectoryGoal() # FollowJointTrajectoryGoal goal


class FollowJointTrajectoryFeedback(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryFeedback.html
    """
    def __init__(self, header=None, joint_names=None, desired=None, actual=None,
                 error=None):
        self.header = header if header else Header()
        self.joint_names = joint_names if joint_names else []
        self.desired = desired if desired else JointTrajectoryPoint()
        self.actual = actual if actual else JointTrajectoryPoint()
        self.error = error if error else JointTrajectoryPoint()


class FollowJointTrajectoryActionFeedback(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryActionFeedback.html
    """
    def __init__(self, header=None, status=None, feedback=None):
        self.header = header if header else Header()
        self.status = status if status else GoalStatus()
        self.feedback = feedback if feedback else FollowJointTrajectoryFeedback()


class FollowJointTrajectoryResult(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryResult.html
    """

    SUCCESSFUL = 0
    INVALID_GOAL = -1
    INVALID_JOINTS = -2
    OLD_HEADER_TIMESTAMP = -3
    PATH_TOLERANCE_VIOLATED = -4
    GOAL_TOLERANCE_VIOLATED = -5

    def __init__(self, error_code=0, error_string=""):
        self.error_code = error_code
        self.error_string = error_string
    
    @classmethod
    def from_msg(cls, msg):
        error_code = msg['error_code']
        return cls(error_code)

    @property
    def human_readable(self):
        cls = type(self)
        for k, v in cls.__dict__.items():
            if v == self.error_code:
                return k
        return ''


class FollowJointTrajectoryActionResult(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html
    """
    def __init__(self, header=None, status=None, result=None):
        self.header = header if header else Header()
        self.status = status if status else GoalStatus()
        self.result = result if result else FollowJointTrajectoryResult()
    
    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        status = GoalStatus.from_msg(msg['status'])
        result = FollowJointTrajectoryResult.from_msg(msg['result'])
        return cls(header, status, result)


from .std_msgs import ROSmsg
from .std_msgs import Header
from .std_msgs import Time
from .actionlib_msgs import GoalID
from .actionlib_msgs import GoalStatus
from .trajectory_msgs import JointTrajectoryPoint
from .trajectory_msgs import JointTrajectory


# C:\Users\rustr\AppData\Local\lxss\rootfs\opt\ros\kinetic\lib\python2.7\dist-packages


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

    def __init__(self, trajectory=JointTrajectory(), path_tolerance=[],
                 goal_tolerance=[], goal_time_tolerance=Time()):
        self.trajectory = trajectory #trajectory_msgs/JointTrajectory 
        self.path_tolerance = path_tolerance #control_msgs/JointTolerance[] 
        self.goal_tolerance = goal_tolerance #control_msgs/JointTolerance[] 
        self.goal_time_tolerance = goal_time_tolerance


class FollowJointTrajectoryActionGoal(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryActionGoal.html
    """

    def __init__(self, header=Header(), goal_id=GoalID(), goal=FollowJointTrajectoryGoal()):
        self.header = header
        self.goal_id = goal_id #actionlib_msgs/GoalID goal_id
        self.goal = goal # FollowJointTrajectoryGoal goal


class FollowJointTrajectoryFeedback(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryFeedback.html
    """
    def __init__(self, header=Header(), joint_names=[], 
                 desired=JointTrajectoryPoint(), actual=JointTrajectoryPoint(),
                 error=JointTrajectoryPoint()):
        self.header = header
        self.joint_names = joint_names
        self.desired = desired
        self.actual = actual
        self.error = error


class FollowJointTrajectoryActionFeedback(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryActionFeedback.html
    """
    def __init__(self, header=Header(), status=GoalStatus(), feedback=FollowJointTrajectoryFeedback()):
        self.header = header
        self.status = status
        self.feedback = feedback


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


class FollowJointTrajectoryActionResult(ROSmsg):
    """http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryActionResult.html
    """
    def __init__(self, header=Header(), status=GoalStatus(), result=FollowJointTrajectoryResult()):
        self.header = header
        self.status = status
        self.result = result
    
    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        status = GoalStatus.from_msg(msg['status'])
        result = FollowJointTrajectoryResult.from_msg(msg['result'])
        return cls(header, status, result)


"""
rostopic info /follow_joint_trajectory/feedback
Type: control_msgs/FollowJointTrajectoryActionFeedback

rostopic info /follow_joint_trajectory/result
Type: control_msgs/FollowJointTrajectoryActionResult

rostopic info /follow_joint_trajectory/goal
Type: control_msgs/FollowJointTrajectoryActionGoal


"""
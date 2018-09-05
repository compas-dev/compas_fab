from __future__ import absolute_import

from .std_msgs import ROSmsg
from .std_msgs import Header
from .std_msgs import Time

from .geometry_msgs import Transform
from .geometry_msgs import Twist

class JointTrajectoryPoint(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html
    """

    def __init__(self, positions=[], velocities=[], accelerations=[], effort=[], time_from_start=Time()):
        self.positions = positions
        self.velocities = velocities
        self.accelerations = accelerations
        self.effort = effort
        self.time_from_start = time_from_start

    @classmethod
    def from_msg(cls, msg):
        time_from_start = Time.from_msg(msg['time_from_start'])
        return cls(msg['positions'], msg['velocities'], msg['accelerations'], msg['effort'], time_from_start)


class JointTrajectory(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectory.html
    """

    def __init__(self, header=Header(), joint_names=[], points=[]):
        self.header = header
        self.joint_names = joint_names
        self.points = points

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        joint_names = msg['joint_names']
        points = [JointTrajectoryPoint.from_msg(item) for item in msg['points']]
        return cls(header, joint_names, points)


class MultiDOFJointTrajectoryPoint(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectoryPoint.html
    """

    def __init__(self, transforms=[], velocities=[], accelerations=[], time_from_start=Time()):
        self.transforms = transforms  # geometry_msgs/Transform[]
        self.velocities = velocities  # geometry_msgs/Twist[]
        self.accelerations = accelerations  # geometry_msgs/Twist[]
        self.time_from_start = time_from_start


class MultiDOFJointTrajectory(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html
    """

    def __init__(self, header=Header(), joint_names=[], points=[]):
        self.header = header
        self.joint_names = joint_names
        self.points = points

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        joint_names = msg['joint_names']
        points = [MultiDOFJointTrajectoryPoint.from_msg(item) for item in msg['points']]
        return cls(header, joint_names, points)
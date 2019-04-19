from __future__ import absolute_import

from .std_msgs import ROSmsg
from .std_msgs import Header
from .std_msgs import Time

from .geometry_msgs import Transform
from .geometry_msgs import Twist


class JointTrajectoryPoint(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html
    """

    def __init__(self, positions=None, velocities=None, accelerations=None, effort=None, time_from_start=None):
        self.positions = positions if positions else []
        self.velocities = velocities if velocities else []
        self.accelerations = accelerations if accelerations else []
        self.effort = effort if effort else []
        self.time_from_start = time_from_start if time_from_start else Time()
        # TODO: check if we need to enter zeros to all

    @classmethod
    def from_msg(cls, msg):
        time_from_start = Time.from_msg(msg['time_from_start'])
        return cls(msg['positions'], msg['velocities'], msg['accelerations'], msg['effort'], time_from_start)

    @property
    def msg(self):
        msg = super(JointTrajectoryPoint, self).msg
        if not len(self.accelerations):
            del msg['accelerations']
        if not len(self.effort):
            del msg['effort']
        return msg


class JointTrajectory(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/JointTrajectory.html
    """

    def __init__(self, header=None, joint_names=None, points=None):
        self.header = header if header else Header()
        self.joint_names = joint_names if joint_names else []
        self.points = points if points else []

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        joint_names = msg['joint_names']
        points = [JointTrajectoryPoint.from_msg(item) for item in msg['points']]
        return cls(header, joint_names, points)


class MultiDOFJointTrajectoryPoint(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectoryPoint.html
    """

    def __init__(self, transforms=None, velocities=None, accelerations=None, time_from_start=None):
        self.transforms = transforms if transforms else []  # geometry_msgs/Transform[]
        self.velocities = velocities if velocities else []  # geometry_msgs/Twist[]
        self.accelerations = accelerations if accelerations else []  # geometry_msgs/Twist[]
        self.time_from_start = time_from_start if time_from_start else Time()


class MultiDOFJointTrajectory(ROSmsg):
    """http://docs.ros.org/kinetic/api/trajectory_msgs/html/msg/MultiDOFJointTrajectory.html
    """

    def __init__(self, header=None, joint_names=None, points=None):
        self.header = header if header else Header()
        self.joint_names = joint_names if joint_names else []
        self.points = points if points else []

    @classmethod
    def from_msg(cls, msg):
        header = Header.from_msg(msg['header'])
        joint_names = msg['joint_names']
        points = [MultiDOFJointTrajectoryPoint.from_msg(item) for item in msg['points']]
        return cls(header, joint_names, points)

from __future__ import absolute_import


class URmsg(object):
    """
    """
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

    def __str__(self):
        result = ""
        for key, value in self.__dict__.items():
            if value:
                result += "%s, " % value
        return result[:-2]
    
    def __repr__(self):
        return self.__str__


class Point(URmsg):
    """
    """
    def __init__(self, x, y, z):
        self.x = x # [m]
        self.y = y # [m]
        self.z = z # [m]

    def __str__(self):
        return '%.6f, %.6f, %.6f' % (self.x ,self.y, self.z)


class AxisAngle(URmsg):
    """
    """
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self):
        return '%.6f, %.6f, %.6f' % (self.x ,self.y, self.z)


class Pose(URmsg):
    """
    """
    def __init__(self, position=Point(0,0,0), orientation=AxisAngle(0,0,0)):
        self.position = position
        self.orientation = orientation

    @classmethod
    def from_frame(cls, frame):
        point = frame.point
        ax, ay, az = frame.axis_angle_vector
        return cls(Point(*list(point)), AxisAngle(ax, ay, az))
    
    def __str__(self):
        return "p[%s, %s]" % (self.position, self.orientation)


class PoseTrajectoryPoint(URmsg):
    """
    """
    def __init__(self, pose=Pose(), acceleration=None, velocity=None, time=None, radius=None):
        self.pose = pose
        self.acceleration = acceleration # [m/s^2]
        self.velocity = velocity # [m/s]
        self.time = time # [s]
        self.radius = radius # [m]
        
    def __str__(self):
        result = "%s" % self.pose
        if self.acceleration:
            result += ", a=%.6f" % self.acceleration
        if self.velocity:
            result += ", v=%.6f" % self.velocity
        if self.time:
            result += ", t=%.6f" % self.time
        if self.radius:
            result += ", r=%.6f" % self.radius
        return result


class MoveInJointSpace(URmsg):
    """
    """
    def __init__(self, pose_trajectory_point=PoseTrajectoryPoint()):
        self.pose_trajectory_point = pose_trajectory_point

    def __str__(self):
        return "movej(%s)" % self.pose_trajectory_point


class MoveInToolSpace(URmsg):
    """
    """
    def __init__(self, pose_trajectory_point=PoseTrajectoryPoint()):
        self.pose_trajectory_point = pose_trajectory_point

    def __str__(self):
        return "movel(%s)" % self.pose_trajectory_point


class URGoal(URmsg):
    """
    """
    def __init__(self, ur_script_lines=[]):
        self.data = [str(line) for line in ur_script_lines]
    
    @property
    def msg(self):
        return {"script_lines": self.data}



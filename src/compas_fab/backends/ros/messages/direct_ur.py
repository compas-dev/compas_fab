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
        self.x = x  # [m]
        self.y = y  # [m]
        self.z = z  # [m]

    def __str__(self):
        return '%.6f, %.6f, %.6f' % (self.x, self.y, self.z)


class AxisAngle(URmsg):
    """
    """

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return '%.6f, %.6f, %.6f' % (self.x, self.y, self.z)


class URPose(URmsg):
    """
    """

    def __init__(self, position=Point(0, 0, 0), orientation=AxisAngle(0, 0, 0)):
        self.position = position
        self.orientation = orientation

    @classmethod
    def from_frame(cls, frame):
        point = frame.point
        ax, ay, az = frame.axis_angle_vector
        return cls(Point(*list(point)), AxisAngle(ax, ay, az))

    def __str__(self):
        return "p[%s, %s]" % (self.position, self.orientation)


class URPoseTrajectoryPoint(URmsg):
    """
    """

    def __init__(self, pose=URPose(), acceleration=None, velocity=None, time=None, radius=None):
        self.pose = pose
        self.acceleration = acceleration  # [m/s^2]
        self.velocity = velocity  # [m/s]
        self.time = time  # [s]
        self.radius = radius  # [m]

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


class URMovej(URmsg):
    """
    """

    def __init__(self, pose_trajectory_point=URPoseTrajectoryPoint()):
        self.pose_trajectory_point = pose_trajectory_point

    def __str__(self):
        return "movej(%s)" % self.pose_trajectory_point


class URMovel(URmsg):
    """
    """

    def __init__(self, pose_trajectory_point=URPoseTrajectoryPoint()):
        self.pose_trajectory_point = pose_trajectory_point

    def __str__(self):
        return "movel(%s)" % self.pose_trajectory_point


class URGoal(URmsg):
    """
    """

    def __init__(self, script_lines=[]):
        self.script = "def prog():\n\t"
        self.script += "\n\t".join([str(line) for line in script_lines])
        self.script += "\nend\nprog()\n\n"

    @property
    def msg(self):
        return {"script": self.script}


if __name__ == "__main__":

    from compas.geometry import Frame
    f1 = Frame((0., -193, 1001.), (-1., 0., 0.), (0., 0., -1.))
    f2 = Frame((0., -193, 709.0), (-1., 0., 0.), (0., 0., -1.))
    f1.point /= 1000.
    f2.point /= 1000.
    frames = [f1, f2]

    acceleration = 0.35
    velocity = 0.17
    time = 5.
    script_lines = []

    for frame in frames:
        ptp = URPoseTrajectoryPoint(URPose.from_frame(
            frame), acceleration, velocity, time, None)
        move = URMovej(ptp)
        script_lines.append(move)

    urgoal = URGoal(script_lines)
    print(urgoal.msg["script_lines"])

from __future__ import absolute_import

from .std_msgs import ROSmsg
from .std_msgs import Header
from .std_msgs import Time


class GoalID(ROSmsg):
    """http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html
    """

    def __init__(self, stamp=Time(), id=""):
        self.stamp = stamp
        self.id = id

    @classmethod
    def from_msg(cls, msg):
        stamp = Time.from_msg(msg['stamp'])
        id = msg['id']
        return cls(stamp, id)


class GoalStatus(ROSmsg):
    """http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
    """

    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9

    def __init__(self, goal_id=GoalID(), status=0, text=''):
        self.goal_id = goal_id
        self.status = status
        self.text = text

    @classmethod
    def from_msg(cls, msg):
        goal_id = GoalID.from_msg(msg['goal_id'])
        status = msg['status']
        text = msg['text']
        return cls(goal_id, status, text)

    @property
    def human_readable(self):
        cls = type(self)
        for k, v in cls.__dict__.items():
            if v == self.status:
                return k
        return ''


class GoalStatusArray(ROSmsg):
    """http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html
    """

    def __init__(self, header=None, status_list=None):
        self.header = header or Header()
        self.status_list = status_list or []


"""
rostopic info /follow_joint_trajectory/cancel
Type: actionlib_msgs/GoalID

rostopic info /follow_joint_trajectory/status
Type: actionlib_msgs/GoalStatusArray

"""

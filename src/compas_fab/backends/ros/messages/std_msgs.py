class ROSmsg(object):
    """The base class for ros messages.
    """

    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)

    @property
    def msg(self):
        msg = {}
        for key, value in self.__dict__.items():
            if hasattr(value, 'msg'):
                msg[key] = value.msg
            elif isinstance(value, list):
                if len(value):
                    if hasattr(value[0], 'msg'):
                        msg[key] = [v.msg for v in value]
                    else:
                        msg[key] = value
                else:
                    msg[key] = value
            else:
                msg[key] = value
        return msg

    @classmethod
    def from_msg(cls, msg):
        return cls(**msg)

    def __str__(self):
        return str(self.msg)

    def __repr__(self):
        return self.__str__


class Time(ROSmsg):
    """http://docs.ros.org/kinetic/api/std_msgs/html/msg/Time.html
    """

    def __init__(self, secs=0., nsecs=0.):
        self.secs = secs
        self.nsecs = nsecs

    def seconds(self):
        return self.secs + 1e-9 * self.nsecs


class Header(ROSmsg):
    """http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """

    def __init__(self, seq=0, stamp=Time(), frame_id='/world'):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id


class String(ROSmsg):
    """http://docs.ros.org/api/std_msgs/html/msg/String.html
    """

    def __init__(self, data=''):
        self.data = data

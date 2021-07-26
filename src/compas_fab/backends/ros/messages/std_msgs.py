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
        args = []
        for key, value in self.__dict__.items():
            args.append('{}={!r}'.format(key, value))

        return '{}({})'.format(self.__class__.__name__, ', '.join(args))


class Time(ROSmsg):
    """https://docs.ros.org/kinetic/api/std_msgs/html/msg/Time.html
    """

    def __init__(self, secs=0, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs

    def seconds(self):
        return self.secs + 1e-9 * self.nsecs


class Header(ROSmsg):
    """https://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
    """

    def __init__(self, seq=0, stamp=Time(), frame_id='/world'):
        self.seq = seq
        self.stamp = stamp
        self.frame_id = frame_id


class String(ROSmsg):
    """https://docs.ros.org/api/std_msgs/html/msg/String.html
    """

    def __init__(self, data=''):
        self.data = data


class MultiArrayDimension(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayDimension.html
    """

    def __init__(self, label=None, size=0, stride=0):
        self.label = label or ""  # label of given dimension
        self.size = size  # size of given dimension (in type units)
        self.stride = stride  # stride of given dimension

    @classmethod
    def from_msg(cls, msg):
        return cls(msg['label'], msg['size'], msg['stride'])


class MultiArrayLayout(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayLayout.html
    """

    def __init__(self, dim=None, data_offset=None):
        self.dim = dim or []
        self.data_offset = data_offset or 0

    @classmethod
    def from_msg(cls, msg):
        dim = msg['dim']
        return cls(dim, msg['data_offset'])


class Int8MultiArray(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html
    """

    def __init__(self, layout=None, data=None):
        self.layout = layout or MultiArrayLayout()
        self.data = data or []

    @classmethod
    def from_msg(cls, msg):
        layout = MultiArrayLayout.from_msg(msg['layout'])
        return cls(layout, msg['data'])


class Float32MultiArray(ROSmsg):
    """http://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html
    """

    def __init__(self, layout=None, data=None):
        self.layout = layout or MultiArrayLayout()
        self.data = data or []

    @classmethod
    def from_msg(cls, msg):
        layout = MultiArrayLayout.from_msg(msg['layout'])
        return cls(layout, msg['data'])


class Int32(ROSmsg):
    """http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Int32.html
    """

    def __init__(self, data=None):
        self.data = data or 0

    @classmethod
    def from_msg(cls, msg):
        return cls(msg['data'])

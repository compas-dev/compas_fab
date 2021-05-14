from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import ROSmsg
from compas_fab.backends.ros.messages import String
from compas_fab.backends.ros.messages import Time


def test_rosmsg_attributes():
    r = ROSmsg(a=1, b=2)
    assert r.a == 1
    assert r.b == 2


def test_rosmsg_str():
    r = ROSmsg(a=1, b='2')
    assert str(r) == str(dict(a=1, b='2'))


def test_nested_repr():
    t = Time(80, 20)
    h = Header(seq=10, stamp=t, frame_id='/wow')
    assert repr(h) == "Header(seq=10, stamp=Time(secs=80, nsecs=20), frame_id='/wow')"


def test_subclasses_define_type_name():
    for cls in ROSmsg.__subclasses__():
        assert cls.ROS_MSG_TYPE is not None, 'Class {} does not define its msg type'.format(cls.__name__)


def test_consistent_naming():
    for cls in ROSmsg.__subclasses__():
        assert cls.ROS_MSG_TYPE.split('/')[1] == cls.__name__, 'Class {} does not match to the ROS msg type name={}'.format(cls.__name__, cls.ROS_MSG_TYPE)


def test_uniqueness_of_msg_type():
    all_types = [(cls.ROS_MSG_TYPE, cls) for cls in ROSmsg.__subclasses__()]

    seen = set()
    dupes = set()
    for msg_type, cls in all_types:
        if msg_type not in seen:
            seen.add(msg_type)
        else:
            dupes.add(cls.__name__)

    assert len(dupes) == 0, 'The classes {} define duplicate ROS MSG TYPEs'.format(str(dupes))


def test_parse_from_json():
    msg = ROSmsg.parse('{"data": "Hello"}', 'std_msgs/String')
    assert isinstance(msg, String)
    assert msg.data == 'Hello'


def test_parse_from_dict():
    msg = ROSmsg.parse(dict(data='Hello'), 'std_msgs/String')
    assert isinstance(msg, String)
    assert msg.data == 'Hello'


def test_parse_unknown_type():
    msg = ROSmsg.parse(dict(something='Hello'), 'std_msgs/Unknown')
    assert isinstance(msg, ROSmsg)
    assert msg.something == 'Hello'

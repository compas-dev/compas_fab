from compas_fab.backends.ros.messages import ROSmsg
from compas_fab.backends.ros.messages import Header
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

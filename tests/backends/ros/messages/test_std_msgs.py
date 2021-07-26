from compas_fab.backends.ros.messages import ROSmsg
from compas_fab.backends.ros.messages import Header
from compas_fab.backends.ros.messages import Time
from compas_fab.backends.ros.messages import Float32MultiArray
from compas_fab.backends.ros.messages import Int8MultiArray
from compas_fab.backends.ros.messages import Pose
from compas_fab.backends.ros.messages import PoseArray


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


def test_float32_multiarray():
    m = Float32MultiArray(data=[1.3, 0.5, 3.4])
    assert(repr(m) == "Float32MultiArray(layout=MultiArrayLayout(dim=[], data_offset=0), data=[1.3, 0.5, 3.4])")


def test_int8_multiarray():
    m = Int8MultiArray(data=[1, 2, 3, 4])
    assert(repr(m) == "Int8MultiArray(layout=MultiArrayLayout(dim=[], data_offset=0), data=[1, 2, 3, 4])")


def test_posearray():
    from compas.geometry import Frame
    p = [Pose.from_frame(f) for f in [Frame.worldXY()]]
    m = PoseArray(header=Header(), poses=p)
    assert(repr(m) == "PoseArray(header=Header(seq=0, stamp=Time(secs=0, nsecs=0), frame_id='/world'), poses=[Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))])")  # noqa E501

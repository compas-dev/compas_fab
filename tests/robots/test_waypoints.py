from compas.geometry import Frame, Point, Vector
from compas_fab.robots import FrameWaypoints, PointAxisWaypoints, TargetMode


def test_frame_waypoints_list_behavior():
    fw = FrameWaypoints([Frame.worldXY()], TargetMode.ROBOT)
    assert len(fw) == 1
    fw.append(Frame.worldZX())
    assert len(fw) == 2

    # Test iteration
    frames = [f for f in fw]
    assert len(frames) == 2
    assert frames[0] == Frame.worldXY()
    assert frames[1] == Frame.worldZX()


def test_point_axis_waypoints_list_behavior():
    pw = PointAxisWaypoints([(Point(0, 0, 0), Vector(1, 0, 0))], TargetMode.ROBOT)
    assert len(pw) == 1
    pw.append((Point(1, 1, 1), Vector(0, 1, 0)))
    assert len(pw) == 2

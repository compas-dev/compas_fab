import pytest

from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import ConstraintSetTarget
from compas_fab.robots import JointConstraint
from compas_fab.robots import OrientationConstraint
from compas_fab.robots import PositionConstraint
from compas_fab.robots import TargetMode

from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints

from compas_robots import Configuration
from compas_robots.model import Joint

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector


@pytest.fixture
def target_frame():
    return Frame(Point(1.0, -2.0, 3.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0))


@pytest.fixture
def tool_coordinate_frame():
    return Frame(Point(0.0, 10.0, 20.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0))


@pytest.fixture
def target_configuration():
    return Configuration.from_prismatic_and_revolute_values(
        [10.0, 20.0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0], ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8"]
    )


@pytest.fixture
def frame_target(target_frame):
    return FrameTarget(
        target_frame=target_frame,
        target_mode=TargetMode.ROBOT,
        target_scale=1.0,
        tolerance_position=0.001,
        tolerance_orientation=0.001,
        name="my testing name",
    )


@pytest.fixture
def point_axis_target():
    return PointAxisTarget(
        target_point=Point(1.0, -2.0, 3.0),
        target_vector=Vector(1.0, -1.0, 0.0),
        target_mode=TargetMode.ROBOT,
        target_scale=1.0,
        tolerance_position=0.001,
        tolerance_orientation=0.001,
        name="my testing name",
    )


@pytest.fixture
def configuration_target(target_configuration):
    return ConfigurationTarget(
        target_configuration=target_configuration,
        tolerance_above=[0.01] * 8,
        tolerance_below=[0.0009] * 8,
        name="my testing name",
    )


def test_serialization_targets(frame_target, point_axis_target, configuration_target):
    # FrameTarget
    nt = FrameTarget.__from_data__(frame_target.__data__)
    assert frame_target.target_frame == nt.target_frame
    assert frame_target.target_mode == nt.target_mode
    assert frame_target.target_scale == nt.target_scale
    assert frame_target.tolerance_position == nt.tolerance_position
    assert frame_target.tolerance_orientation == nt.tolerance_orientation
    assert frame_target.name == nt.name

    # PointAxisTarget
    nt = PointAxisTarget.__from_data__(point_axis_target.__data__)
    assert point_axis_target.target_point == nt.target_point
    assert point_axis_target.target_z_axis == nt.target_z_axis
    assert point_axis_target.target_mode == nt.target_mode
    assert point_axis_target.target_scale == nt.target_scale
    assert point_axis_target.tolerance_position == nt.tolerance_position
    assert point_axis_target.tolerance_orientation == nt.tolerance_orientation
    assert point_axis_target.name == nt.name

    # ConfigurationTarget
    nt = ConfigurationTarget.__from_data__(configuration_target.__data__)
    assert configuration_target.target_configuration.close_to(nt.target_configuration)
    assert configuration_target.tolerance_above == nt.tolerance_above
    assert configuration_target.tolerance_below == nt.tolerance_below
    assert configuration_target.name == nt.name


def test_serialization_constraint_sets(target_frame, target_configuration, tool_coordinate_frame):
    tolerance_above = [0.01] * 8
    tolerance_below = [0.0009] * 8
    name = "my testing name"

    # ConstraintSetTarget with JointConstraint
    constraint_set = JointConstraint.joint_constraints_from_configuration(
        target_configuration, tolerance_above, tolerance_below
    )
    target = ConstraintSetTarget(constraint_set, name)
    nt = ConstraintSetTarget.__from_data__(target.__data__)
    assert target.constraint_set == nt.constraint_set
    assert target.name == nt.name
    for c1, c2 in zip(target.constraint_set, nt.constraint_set):
        assert c1.__data__ == c2.__data__

    # ConstraintSetTarget with OrientationConstraint and PositionConstraint
    link_name = "tool0"
    tolerances_orientation = [0.0123] * 3
    orientation_constraint_weight = 0.789
    orientation_constraint = OrientationConstraint.from_frame(
        target_frame, tolerances_orientation, link_name, tool_coordinate_frame, orientation_constraint_weight
    )
    position_constraint_weight = 0.456
    tolerances_position = 0.567
    position_constraint = PositionConstraint.from_frame(
        target_frame, tolerances_position, link_name, tool_coordinate_frame, position_constraint_weight
    )
    target = ConstraintSetTarget([orientation_constraint, position_constraint], name)
    nt = ConstraintSetTarget.__from_data__(target.__data__)
    assert target.constraint_set == nt.constraint_set
    assert target.name == nt.name
    for c1, c2 in zip(target.constraint_set, nt.constraint_set):
        assert c1.__data__ == c2.__data__


@pytest.fixture
def frame_waypoints():
    target_frames = []
    target_frames.append(Frame(Point(1.0, -2.0, 3.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0)))
    target_frames.append(Frame(Point(4.0, -5.0, 6.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0)))
    target_frames.append(Frame(Point(7.0, -8.0, 9.0), Vector(-1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0)))

    return FrameWaypoints(
        target_frames=target_frames,
        target_mode=TargetMode.ROBOT,
        target_scale=1.0,
        tolerance_position=0.001,
        tolerance_orientation=0.001,
        name="my testing waypoints",
    )


@pytest.fixture
def point_axis_waypoints():
    target_points_and_axes = []
    target_points_and_axes.append((Point(1.0, -2.0, 3.0), Vector(1.0, 0.0, 0.0)))
    target_points_and_axes.append((Point(4.0, -5.0, 6.0), Vector(1.0, 0.0, 0.0)))
    target_points_and_axes.append((Point(7.0, -8.0, 9.0), Vector(-1.0, 0.0, 0.0)))
    return PointAxisWaypoints(
        target_points_and_axes=target_points_and_axes,
        target_mode=TargetMode.ROBOT,
        target_scale=1.0,
        tolerance_position=0.001,
        tolerance_orientation=0.001,
        name="my testing waypoints",
    )


def test_serialization_frame_waypoints(frame_waypoints):
    # FrameWaypoints
    nt = FrameWaypoints.__from_data__(frame_waypoints.__data__)
    for f1, f2 in zip(frame_waypoints.target_frames, nt.target_frames):
        assert f1 == f2
    assert frame_waypoints.target_mode, nt.target_mode
    assert frame_waypoints.target_scale, nt.target_scale
    assert frame_waypoints.tolerance_position, nt.tolerance_position
    assert frame_waypoints.tolerance_orientation, nt.tolerance_orientation
    assert frame_waypoints.name == nt.name


def test_serialization_frame_waypoints(point_axis_waypoints):
    # PointAxisWaypoints
    nt = PointAxisWaypoints.__from_data__(point_axis_waypoints.__data__)
    for (p1, a1), (p2, a2) in zip(point_axis_waypoints.target_points_and_axes, nt.target_points_and_axes):
        assert p1 == p2
        assert a1 == a2
    assert point_axis_waypoints.target_mode, nt.target_mode
    assert point_axis_waypoints.target_scale, nt.target_scale
    assert point_axis_waypoints.tolerance_position, nt.tolerance_position
    assert point_axis_waypoints.tolerance_orientation, nt.tolerance_orientation
    assert point_axis_waypoints.name == nt.name


def test_target_scale(frame_target):
    scale_factor = 0.001
    target = frame_target.copy()
    target.target_scale = scale_factor
    nt = target.normalized_to_meters()
    assert nt.target_frame == target.target_frame.scaled(scale_factor)
    assert nt.tolerance_position == target.tolerance_position * scale_factor
    assert nt.tolerance_orientation == target.tolerance_orientation  # No Scaling


def test_point_axis_target_scale(point_axis_target):
    scale_factor = 0.001
    target = point_axis_target.copy()
    target.target_scale = scale_factor
    nt = target.normalized_to_meters()
    assert nt.target_point == target.target_point.scaled(scale_factor)
    assert nt.target_z_axis == target.target_z_axis  # No Scaling
    assert nt.tolerance_position == target.tolerance_position * scale_factor
    assert nt.tolerance_orientation == target.tolerance_orientation  # No Scaling


def test_frame_waypoints_scale(frame_waypoints):
    scale_factor = 0.001
    waypoint = frame_waypoints.copy()
    waypoint.target_scale = scale_factor
    nt = waypoint.normalized_to_meters()
    assert nt.tolerance_position == waypoint.tolerance_position * scale_factor
    assert nt.tolerance_orientation == waypoint.tolerance_orientation  # No Scaling
    for f1, f2 in zip(waypoint.target_frames, nt.target_frames):
        assert f1.scaled(scale_factor) == f2


def test_point_axis_waypoints_scale(point_axis_waypoints):
    scale_factor = 0.001
    waypoint = point_axis_waypoints.copy()
    waypoint.target_scale = scale_factor
    nt = waypoint.normalized_to_meters()
    assert nt.tolerance_position == waypoint.tolerance_position * scale_factor
    for (p1, a1), (p2, a2) in zip(waypoint.target_points_and_axes, nt.target_points_and_axes):
        assert p1.scaled(scale_factor) == p2
        assert a1 == a2

import pytest

from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import ConstraintSetTarget
from compas_fab.robots import Waypoints
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import PointAxisWaypoints

from compas_robots import Configuration

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector

from compas.tolerance import TOL


@pytest.fixture
def target_frame():
    return Frame(Point(1.0, -2.0, 3.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0))


@pytest.fixture
def tool_coordinate_frame():
    return Frame(Point(0.0, 10.0, 20.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0))


def test_serialization(target_frame, tool_coordinate_frame):
    tolerance_position = 0.001
    tolerance_orientation = 0.001
    name = "my testing name"

    # FrameTarget
    target = FrameTarget(target_frame, tolerance_position, tolerance_orientation, tool_coordinate_frame, name)
    nt = FrameTarget.__from_data__(target.__data__)
    assert target.target_frame == nt.target_frame
    assert target.tool_coordinate_frame == nt.tool_coordinate_frame
    assert TOL.is_close(target.tolerance_position, nt.tolerance_position)
    assert TOL.is_close(target.tolerance_orientation, nt.tolerance_orientation)
    assert target.name == nt.name

    # PointAxisTarget
    target_point = Point(1.0, -2.0, 3.0)
    target_vector = Vector(1.0, -1.0, 0.0)
    target = PointAxisTarget(target_point, target_vector, tolerance_position, tool_coordinate_frame, name)
    nt = PointAxisTarget.__from_data__(target.__data__)
    assert target.target_point == nt.target_point
    assert target.target_z_axis == nt.target_z_axis
    assert target.tool_coordinate_frame == nt.tool_coordinate_frame
    assert TOL.is_close(target.tolerance_position, nt.tolerance_position)
    assert target.name == nt.name

    # ConfigurationTarget
    target_configuration = Configuration.from_prismatic_and_revolute_values(
        [10.0, 20.0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    )
    tolerance_above = [0.01] * 8
    tolerance_below = [0.0009] * 8
    target = ConfigurationTarget(target_configuration, tolerance_above, tolerance_below, name)
    nt = ConfigurationTarget.__from_data__(target.__data__)
    assert target.target_configuration.close_to(nt.target_configuration)
    assert target.tolerance_above == nt.tolerance_above
    assert target.tolerance_below == nt.tolerance_below
    assert target.name == nt.name

    # ConstraintSetTarget

    # FrameWaypoints

    # PointAxisWaypoints

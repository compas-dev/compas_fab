from compas_fab.robots import RobotLibrary
import pytest


@pytest.fixture
def all_robots():
    robots = []
    robots.append(RobotLibrary.rfl())
    robots.append(RobotLibrary.ur5())
    robots.append(RobotLibrary.ur10e())
    robots.append(RobotLibrary.abb_irb4600_40_255())
    return robots


def test_robot_semantics_and_geometry(all_robots):
    for robot in all_robots:
        robot.ensure_semantics()
        robot.ensure_geometry()
        assert robot.name
        robot.info()

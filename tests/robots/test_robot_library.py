from compas_fab.robots import RobotCellLibrary
import pytest


@pytest.fixture
def all_robots_cells_and_states():
    robots = []
    robots.append(RobotCellLibrary.rfl())
    robots.append(RobotCellLibrary.ur5())
    robots.append(RobotCellLibrary.ur10e())
    robots.append(RobotCellLibrary.abb_irb4600_40_255())
    return robots


def test_robot_semantics_and_geometry(all_robots_cells_and_states):
    for robot_cell, robot_state in all_robots_cells_and_states:
        robot_cell.ensure_semantics()
        robot_cell.ensure_geometry()
        assert robot_cell.robot_model.name
        robot_cell.print_info()

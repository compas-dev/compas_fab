import pytest
from compas import IPY

from compas_fab.robots import RobotCellLibrary

if not IPY:
    from compas_fab.backends import PyBulletClient
    from compas_fab.backends import PyBulletPlanner


@pytest.fixture
def pybullet_client():
    with PyBulletClient(connection_type="direct") as client:
        yield client


def test_set_robot_cell(pybullet_client):
    robot_cell1, robot_cell_state1 = RobotCellLibrary.ur5_cone_tool()
    robot_cell2, robot_cell_state2 = RobotCellLibrary.abb_irb4600_40_255_gripper_one_beam()
    planner = PyBulletPlanner(pybullet_client)
    planner.set_robot_cell(robot_cell1, robot_cell_state1)

    # Use the same planner to set a new robot cell
    planner.set_robot_cell(robot_cell2, robot_cell_state2)
    robot_cell3, robot_cell_state3 = RobotCellLibrary.ur5_cone_tool()
    planner.set_robot_cell(robot_cell3, robot_cell_state3)

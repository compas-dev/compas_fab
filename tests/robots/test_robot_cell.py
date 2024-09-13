from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotLibrary
from compas_robots import RobotModel

from copy import deepcopy


def test_copy():
    """Test to comply with Data.copy() mechanism"""

    robot_cell, robot_cell_state = RobotCellLibrary.ur10e_gripper_one_beam()

    robot_cell_copy = robot_cell.copy()
    assert robot_cell.robot.name == robot_cell_copy.robot.name
    assert robot_cell.tool_ids == robot_cell_copy.tool_ids
    assert robot_cell.rigid_body_ids == robot_cell_copy.rigid_body_ids

    robot_cell_state_copy = robot_cell_state.copy()
    assert robot_cell_state.tool_ids == robot_cell_state_copy.tool_ids
    assert robot_cell_state.rigid_body_ids == robot_cell_state_copy.rigid_body_ids


def test_deepcopy():
    """Test to make sure copy.deepcopy works"""
    robot_cell, robot_cell_state = RobotCellLibrary.ur10e_gripper_one_beam()
    robot_cell_copy = deepcopy(robot_cell)

    assert robot_cell.robot.name == robot_cell_copy.robot.name
    assert robot_cell.tool_ids == robot_cell_copy.tool_ids
    assert robot_cell.rigid_body_ids == robot_cell_copy.rigid_body_ids

    robot_cell_state_copy = deepcopy(robot_cell_state)
    assert robot_cell_state.tool_ids == robot_cell_state_copy.tool_ids
    assert robot_cell_state.rigid_body_ids == robot_cell_state_copy.rigid_body_ids


if __name__ == "__main__":
    test_copy_robot()

from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotLibrary
from compas_robots import RobotModel

import pytest

from copy import deepcopy


@pytest.fixture
def ur10e_gripper_one_beam():
    return RobotCellLibrary.ur10e_gripper_one_beam()


@pytest.fixture
def abb_irb4600_40_255_gripper_one_beam():
    return RobotCellLibrary.abb_irb4600_40_255_gripper_one_beam()


@pytest.fixture
def abb_irb4600_40_255_printing_tool():
    return RobotCellLibrary.abb_irb4600_40_255_printing_tool()


def test_robot_cell_copy(ur10e_gripper_one_beam, abb_irb4600_40_255_gripper_one_beam, abb_irb4600_40_255_printing_tool):
    """Test to comply with Data.copy() mechanism"""

    def test(robot_cell, robot_cell_state):
        robot_cell_copy = robot_cell.copy()
        assert robot_cell.robot.name == robot_cell_copy.robot.name
        assert robot_cell.tool_ids == robot_cell_copy.tool_ids
        assert robot_cell.rigid_body_ids == robot_cell_copy.rigid_body_ids

        robot_cell_state_copy = robot_cell_state.copy()
        assert robot_cell_state.tool_ids == robot_cell_state_copy.tool_ids
        assert robot_cell_state.rigid_body_ids == robot_cell_state_copy.rigid_body_ids

    test(*ur10e_gripper_one_beam)
    test(*abb_irb4600_40_255_gripper_one_beam)
    test(*abb_irb4600_40_255_printing_tool)


def test_robot_cell_deepcopy(
    ur10e_gripper_one_beam, abb_irb4600_40_255_gripper_one_beam, abb_irb4600_40_255_printing_tool
):
    """Test to make sure copy.deepcopy works"""

    def test(robot_cell, robot_cell_state):

        robot_cell_copy = deepcopy(robot_cell)
        assert robot_cell.robot.name == robot_cell_copy.robot.name
        assert robot_cell.tool_ids == robot_cell_copy.tool_ids
        assert robot_cell.rigid_body_ids == robot_cell_copy.rigid_body_ids

        robot_cell_state_copy = deepcopy(robot_cell_state)
        assert robot_cell_state.tool_ids == robot_cell_state_copy.tool_ids
        assert robot_cell_state.rigid_body_ids == robot_cell_state_copy.rigid_body_ids

    test(*ur10e_gripper_one_beam)
    test(*abb_irb4600_40_255_gripper_one_beam)
    test(*abb_irb4600_40_255_printing_tool)


# TODO: test remaining functions within RobotCell

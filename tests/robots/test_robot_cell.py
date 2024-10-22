from copy import deepcopy

import pytest
from compas import IPY

from compas_fab.robots import RobotCell  # noqa: F401
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState  # noqa: F401

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Tuple


@pytest.fixture
def ur5():
    return RobotCellLibrary.ur5()


@pytest.fixture
def ur10e_gripper_one_beam():
    return RobotCellLibrary.ur10e_gripper_one_beam()


@pytest.fixture
def abb_irb4600_40_255_gripper_one_beam():
    return RobotCellLibrary.abb_irb4600_40_255_gripper_one_beam()


@pytest.fixture
def abb_irb4600_40_255_printing_tool():
    return RobotCellLibrary.abb_irb4600_40_255_printing_tool()


def test_init(abb_irb4600_40_255_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    """Test to make sure the RobotCell object is initialized correctly"""

    robot_cell, robot_cell_state = abb_irb4600_40_255_gripper_one_beam
    new_robot_cell = RobotCell(
        robot_cell.robot_model,
        robot_cell.robot_semantics,
        robot_cell.tool_models,
        robot_cell.rigid_body_models,
    )
    assert new_robot_cell.robot_model.name == robot_cell.robot_model.name
    assert new_robot_cell.robot_semantics.main_group_name == robot_cell.robot_semantics.main_group_name
    assert new_robot_cell.tool_models == robot_cell.tool_models
    assert new_robot_cell.rigid_body_models == robot_cell.rigid_body_models


def test_hash(abb_irb4600_40_255_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    """Test to make sure the RobotCell object is hashable"""

    robot_cell, robot_cell_state = abb_irb4600_40_255_gripper_one_beam
    robot_cell.sha256()


def test_copy(ur10e_gripper_one_beam, abb_irb4600_40_255_gripper_one_beam, abb_irb4600_40_255_printing_tool):
    """Test to comply with Data.copy() mechanism"""

    def test(robot_cell, robot_cell_state):
        # type: (RobotCell, RobotCellState) -> None
        robot_cell_copy = robot_cell.copy()  # type: RobotCell
        assert robot_cell.robot_model.name == robot_cell_copy.robot_model.name
        assert robot_cell.robot_semantics.main_group_name == robot_cell_copy.robot_semantics.main_group_name
        assert robot_cell.tool_ids == robot_cell_copy.tool_ids
        assert robot_cell.rigid_body_ids == robot_cell_copy.rigid_body_ids

        robot_cell_state_copy = robot_cell_state.copy()  # type: RobotCellState
        assert robot_cell_state.tool_ids == robot_cell_state_copy.tool_ids
        assert robot_cell_state.rigid_body_ids == robot_cell_state_copy.rigid_body_ids

        # TODO: Fix RobotModel serialization so that we can make the following assertions
        # assert robot_cell.robot_model.sha256() == robot_cell_copy.robot_model.sha256()
        # assert robot_cell.robot_semantics.sha256() == robot_cell_copy.robot_semantics.sha256()
        # for tool_id in robot_cell.tool_ids:
        #     assert robot_cell.tool_models[tool_id].sha256() == robot_cell_copy.tool_models[tool_id].sha256()
        # for rigid_body_id in robot_cell.rigid_body_ids:
        #     assert (
        #         robot_cell.rigid_body_models[rigid_body_id].sha256()
        #         == robot_cell_copy.rigid_body_models[rigid_body_id].sha256()
        #     )

        # assert robot_cell.sha256() == robot_cell_copy.sha256()
        # assert robot_cell_state.sha256() == robot_cell_state_copy.sha256()

    test(*ur10e_gripper_one_beam)
    test(*abb_irb4600_40_255_gripper_one_beam)
    test(*abb_irb4600_40_255_printing_tool)


def test_deepcopy(ur10e_gripper_one_beam, abb_irb4600_40_255_gripper_one_beam, abb_irb4600_40_255_printing_tool):
    """Test to make sure copy.deepcopy works"""

    def test(robot_cell, robot_cell_state):
        # type: (RobotCell, RobotCellState) -> None

        robot_cell_copy = deepcopy(robot_cell)
        assert robot_cell.robot_model.name == robot_cell_copy.robot_model.name
        assert robot_cell.robot_semantics.main_group_name == robot_cell_copy.robot_semantics.main_group_name
        assert robot_cell.tool_ids == robot_cell_copy.tool_ids
        assert robot_cell.rigid_body_ids == robot_cell_copy.rigid_body_ids

        robot_cell_state_copy = deepcopy(robot_cell_state)
        assert robot_cell_state.tool_ids == robot_cell_state_copy.tool_ids
        assert robot_cell_state.rigid_body_ids == robot_cell_state_copy.rigid_body_ids

        assert robot_cell.sha256() == robot_cell_copy.sha256()
        assert robot_cell.robot_model.sha256() == robot_cell_copy.robot_model.sha256()
        assert robot_cell.robot_semantics.sha256() == robot_cell_copy.robot_semantics.sha256()
        for tool_id in robot_cell.tool_ids:
            assert robot_cell.tool_models[tool_id].sha256() == robot_cell_copy.tool_models[tool_id].sha256()
        for rigid_body_id in robot_cell.rigid_body_ids:
            assert (
                robot_cell.rigid_body_models[rigid_body_id].sha256()
                == robot_cell_copy.rigid_body_models[rigid_body_id].sha256()
            )

        assert robot_cell_state.sha256() == robot_cell_state_copy.sha256()

    test(*ur10e_gripper_one_beam)
    test(*abb_irb4600_40_255_gripper_one_beam)
    test(*abb_irb4600_40_255_printing_tool)


# TODO: test remaining functions within RobotCell

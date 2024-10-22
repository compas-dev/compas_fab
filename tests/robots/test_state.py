# Testing for the states module

from copy import deepcopy

import pytest
from compas import IPY
from compas.data import Data
from compas.geometry import Frame
from compas.geometry import Transformation

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas_robots import Configuration  # noqa: F401

        from compas_fab.robots import RobotCell  # noqa: F401

from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState
from compas_fab.robots import ToolState
from compas_robots import Configuration

# In this file context
# - rbs stands for RigidBodyState
# - rcs stands for RobotCellState
# - ts stands for ToolState

# --------------------------------
# Tests for Rigid Body State (RBS)
# --------------------------------


@pytest.fixture
def rbs_stationary():
    # type: () -> RigidBodyState
    rbs = RigidBodyState(
        frame=Frame([1, 2, 3], [0.1, 0.2, 0.3]),
    )
    return rbs


@pytest.fixture
def rbs_attached_to_tool():
    # type: () -> RigidBodyState
    rbs = RigidBodyState(
        frame=None,
        attachment_frame=Frame([1, 2, 3], [0.1, 0.2, 0.3]),
        attached_to_tool="tool0",
        touch_links=["link0", "link1"],
        touch_bodies=["body0", "body1"],
    )
    return rbs


@pytest.fixture
def rbs_attached_to_link():
    # type: () -> RigidBodyState
    rbs = RigidBodyState(
        frame=None,
        attachment_frame=Frame([1, 2, 3], [0.1, 0.2, 0.3]),
        attached_to_link="link0",
        touch_links=["link0", "link1"],
        touch_bodies=["body0", "body1"],
    )
    return rbs


def test_rbs_eq(rbs_stationary):
    """Test to make sure the RigidBodyState object is comparable."""
    a = rbs_stationary
    b = deepcopy(rbs_stationary)
    assert a == b
    b.frame = Frame([3, 2, 1], [0.1, 0.2, 0.3])
    assert a != b
    b = deepcopy(rbs_stationary)
    b.attached_to_link = "victor"
    assert a != b
    b = deepcopy(rbs_stationary)
    b.attached_to_tool = "victor"
    assert a != b
    b = deepcopy(rbs_stationary)
    b.touch_links = ["victor", "gonzalo"]
    assert a != b
    b = deepcopy(rbs_stationary)
    b.touch_bodies = ["victor", "gonzalo"]
    assert a != b
    b = deepcopy(rbs_stationary)
    b.attachment_frame = Frame([3, 2, 1], [0.1, 0.2, 0.3])
    assert a != b
    b = deepcopy(rbs_stationary)
    b.is_hidden = True
    assert a != b


def test_rbs_invalid_init():
    # type: () -> None
    """Test to make sure the RigidBodyState object is not initialized with invalid parameters."""
    with pytest.raises(ValueError):
        RigidBodyState(
            frame=Frame([1, 2, 3], [0.1, 0.2, 0.3]),
            attached_to_link="link0",
            attached_to_tool="tool0",
        )


def test_rbs_init_with_transformation():
    # type: () -> None
    """Test to make sure the RigidBodyState object can be initialized with a transformation."""
    frame = Frame([4, 5, 6], [0.1, 0.2, 0.3])
    transformation = Transformation.from_frame(frame)
    # Test frame input with transformation
    rbs = RigidBodyState(frame=transformation)
    assert rbs.frame == frame
    # Test attachment_frame input with transformation
    rbs = RigidBodyState(frame=None, attachment_frame=transformation)
    assert rbs.attachment_frame == frame


def test_rbs_copy(rbs_stationary, rbs_attached_to_tool, rbs_attached_to_link):
    # type: (RigidBodyState, RigidBodyState, RigidBodyState) -> None
    """Test to make sure the RigidBodyState object is copyable"""

    def _copy_test(rbs):
        # type: (RigidBodyState) -> None
        new_rbs = rbs.copy()  # type: RigidBodyState
        assert new_rbs is not rbs
        assert new_rbs == rbs

    _copy_test(rbs_stationary)
    _copy_test(rbs_attached_to_tool)
    _copy_test(rbs_attached_to_link)


def test_rbs_deepcopy(rbs_stationary, rbs_attached_to_tool, rbs_attached_to_link):
    # type: (RigidBodyState, RigidBodyState, RigidBodyState) -> None
    """Test to make sure the RigidBodyState object is copyable"""

    def _copy_test(rbs):
        # type: (RigidBodyState) -> None
        new_rbs = deepcopy(rbs)  # type: RigidBodyState
        assert new_rbs is not rbs
        assert new_rbs == rbs

        # NOTE: Deep copy should result in the same object hash (copy() may not)
        assert new_rbs.sha256() == rbs.sha256()

    _copy_test(rbs_stationary)
    _copy_test(rbs_attached_to_tool)
    _copy_test(rbs_attached_to_link)


# -------------------------
# Tests for Tool State (TS)
# -------------------------


@pytest.fixture
def ts_stationary():
    # type: () -> ToolState
    ts = ToolState(
        frame=Frame([1, 2, 3], [0.1, 0.2, 0.3]),
    )
    return ts


@pytest.fixture
def ts_attached_to_group():
    # type: () -> ToolState
    ts = ToolState(
        frame=None,
        attachment_frame=Frame([1, 2, 3], [0.1, 0.2, 0.3]),
        attached_to_group="group0",
        touch_links=["link0", "link1"],
    )
    return ts


@pytest.fixture
def ts_with_configuration():
    # type: () -> ToolState
    ts = ToolState(
        frame=None,
        attachment_frame=Frame([1, 2, 3], [0.1, 0.2, 0.3]),
        attached_to_group="group0",
        configuration=Configuration.from_prismatic_and_revolute_values(
            [1, 2, 3], [0.1, 0.2, 0.3], ["j0", "j1", "j2", "j3", "j4", "j5"]
        ),
    )
    return ts


def test_ts_eq(ts_stationary):
    # type: (ToolState) -> None
    """Test to make sure the ToolState object is comparable."""
    a = ts_stationary
    b = deepcopy(ts_stationary)
    assert a == b
    b.frame = Frame([3, 2, 1], [0.1, 0.2, 0.3])
    assert a != b
    b = deepcopy(ts_stationary)
    b.attached_to_group = "victor"
    assert a != b
    b = deepcopy(ts_stationary)
    b.touch_links = ["victor", "gonzalo"]
    assert a != b
    b = deepcopy(ts_stationary)
    b.attachment_frame = Frame([3, 2, 1], [0.1, 0.2, 0.3])
    assert a != b
    b = deepcopy(ts_stationary)
    b.configuration = Configuration.from_prismatic_and_revolute_values([1], [0.1], ["j0", "j1"])
    c = deepcopy(ts_stationary)
    c.configuration = Configuration.from_prismatic_and_revolute_values([1], [0.1], ["j0", "j1"])
    assert a != b
    assert b == c
    c.configuration = Configuration.from_prismatic_and_revolute_values([1], [0.9], ["j0", "j1"])
    assert b != c
    c.configuration = None
    assert b != c
    b = deepcopy(ts_stationary)
    b.is_hidden = True
    assert a != b


def test_ts_init_with_transformation():
    # type: () -> None
    """Test to make sure the ToolState object can be initialized with a transformation."""
    frame = Frame([4, 5, 6], [0.1, 0.2, 0.3])
    transformation = Transformation.from_frame(frame)
    # Test frame input with transformation
    ts = ToolState(frame=transformation)
    assert ts.frame == frame
    # Test attachment_frame input with transformation
    ts = ToolState(frame=None, attachment_frame=transformation)
    assert ts.attachment_frame == frame


def test_ts_copy(ts_stationary, ts_attached_to_group, ts_with_configuration):
    # type: (ToolState, ToolState, ToolState) -> None
    """Test to make sure the ToolState object is copyable"""

    def _copy_test(ts):
        # type: (ToolState) -> None
        new_ts = ts.copy()  # type: ToolState
        assert new_ts is not ts
        assert new_ts == ts

    _copy_test(ts_stationary)
    _copy_test(ts_attached_to_group)
    _copy_test(ts_with_configuration)


def test_ts_deepcopy(ts_stationary, ts_attached_to_group, ts_with_configuration):
    # type: (ToolState, ToolState, ToolState) -> None
    """Test to make sure the ToolState object is copyable"""

    def _copy_test(ts):
        # type: (ToolState) -> None
        new_ts = deepcopy(ts)  # type: ToolState
        assert new_ts is not ts
        assert new_ts == ts

        # NOTE: Deep copy should result in the same object hash (copy() may not)
        assert new_ts.sha256() == ts.sha256()

    _copy_test(ts_stationary)
    _copy_test(ts_attached_to_group)
    _copy_test(ts_with_configuration)


# --------------------------------
# Tests for Robot Cell State (RCS)
# --------------------------------


@pytest.fixture
def rcs_ur5():
    # type: () -> RobotCellState
    rc, rcs = RobotCellLibrary.ur5()
    return rcs


@pytest.fixture
def rc_rcs_ur10e_gripper_one_beam():
    # type: () -> Tuple[RobotCell, RobotCellState]
    rc, rcs = RobotCellLibrary.ur10e_gripper_one_beam()
    return rc, rcs


@pytest.fixture
def rcs_ur10e_gripper_one_beam(rc_rcs_ur10e_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> RobotCellState
    rc, rcs = rc_rcs_ur10e_gripper_one_beam
    return rcs


@pytest.fixture
def rc_ur10e_gripper_one_beam(rc_rcs_ur10e_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> RobotCellState
    rc, rcs = rc_rcs_ur10e_gripper_one_beam
    return rc


def test_rcs_eq(rcs_ur10e_gripper_one_beam):
    # type: (RobotCellState) -> None
    """Test to make sure the RobotCellState object is comparable."""
    a = rcs_ur10e_gripper_one_beam
    b = deepcopy(a)
    assert a is not b
    assert a == b

    b = deepcopy(a)
    b.robot_flange_frame = Frame([3, 2, 1], [0.1, 0.2, 0.3])
    assert a != b
    b.robot_flange_frame = None
    assert a != b

    b = deepcopy(a)
    b.robot_configuration.joint_values[0] = 999
    assert a != b
    b.robot_configuration = None
    assert a != b

    tool_id = a.tool_ids[0]
    b = deepcopy(a)
    b.tool_states[tool_id].attachment_frame = Frame([3, 2, 1], [0.1, 0.2, 0.3])
    assert a != b
    b.tool_states[tool_id] = None
    assert a != b
    del b.tool_states[tool_id]
    assert a != b

    rb_id = a.rigid_body_ids[0]
    b = deepcopy(a)
    b.rigid_body_states[rb_id].attachment_frame = Frame([3, 2, 1], [0.1, 0.2, 0.3])
    assert a != b
    b.rigid_body_states[rb_id] = None
    assert a != b
    del b.rigid_body_states[rb_id]
    assert a != b


def test_rcs_copy(rcs_ur5, rcs_ur10e_gripper_one_beam):
    # type: (RobotCellState, RobotCellState) -> None
    """Test to make sure the RobotCellState object is copyable"""

    def _copy_test(rcs):
        # type: (RobotCellState) -> None
        new_rcs = rcs.copy()  # type: RobotCellState
        assert new_rcs is not rcs
        assert new_rcs == rcs

    _copy_test(rcs_ur5)
    _copy_test(rcs_ur10e_gripper_one_beam)


def test_rcs_deepcopy(rcs_ur5, rcs_ur10e_gripper_one_beam):
    # type: (RobotCellState, RobotCellState) -> None
    """Test to make sure the RobotCellState object is copyable"""

    def _copy_test(rcs):
        # type: (RobotCellState) -> None
        new_rcs = deepcopy(rcs)  # type: RobotCellState
        assert new_rcs is not rcs
        assert new_rcs == rcs

        # NOTE: Deep copy should result in the same object hash (copy() may not)
        assert new_rcs.sha256() == rcs.sha256()

    _copy_test(rcs_ur5)
    _copy_test(rcs_ur10e_gripper_one_beam)


def test_rcs_get_ids(rc_rcs_ur10e_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    """Testing the get id functions."""
    rc, rcs = rc_rcs_ur10e_gripper_one_beam
    group = rc.main_group_name

    gripper_id = list(rc.tool_models.keys())[0]
    beam_id = list(rc.rigid_body_models.keys())[0]
    assert rcs.get_attached_tool_id(group) == gripper_id
    assert rcs.get_detached_tool_ids() == []
    assert rcs.get_attached_rigid_body_ids() == [beam_id]
    assert rcs.get_attached_workpiece_ids(group) == [beam_id]


def test_rcs_set_attach_functions(rc_rcs_ur10e_gripper_one_beam, ts_stationary):
    # type: (Tuple[RobotCell, RobotCellState], ToolState) -> None
    """Testing the set attach functions."""
    rc, rcs = rc_rcs_ur10e_gripper_one_beam
    rcs = deepcopy(rcs)
    group = rc.main_group_name

    gripper_id = list(rc.tool_models.keys())[0]
    beam_id = list(rc.rigid_body_models.keys())[0]
    rcs.set_tool_attached_to_group(gripper_id, group)

    assert rcs.get_attached_tool_id(group) == gripper_id
    assert rcs.tool_states[gripper_id].attached_to_group == group
    assert rcs.tool_states[gripper_id].frame == None
    assert rcs.tool_states[gripper_id].attachment_frame == Frame.worldXY()
    assert rcs.tool_states[gripper_id].touch_links == []

    # Detach tool manually
    rcs.tool_states[gripper_id].attached_to_group = None
    assert rcs.get_attached_tool_id(group) == None
    assert rcs.get_detached_tool_ids() == [gripper_id]
    # Attach tool to group using function
    attachment_frame = Frame([1, 2, 3], [0.1, 0.2, 0.3])
    rcs.set_tool_attached_to_group(gripper_id, group, attachment_frame=attachment_frame, touch_links=["link0", "link1"])
    assert rcs.get_attached_tool_id(group) == gripper_id
    assert rcs.tool_states[gripper_id].attached_to_group == group
    assert rcs.tool_states[gripper_id].attachment_frame == attachment_frame
    assert rcs.tool_states[gripper_id].touch_links == ["link0", "link1"]

    # Add another tool state into the robot_cell_state
    print_tool_id = "print_tool"
    rcs.tool_states[print_tool_id] = ts_stationary
    assert rcs.get_detached_tool_ids() == [print_tool_id]
    assert set(rcs.tool_ids) == set([gripper_id, print_tool_id])

    # Attach the print tool, the gripper should be detached automatically
    rcs.set_tool_attached_to_group(print_tool_id, group)
    assert rcs.get_attached_tool_id(group) == print_tool_id
    assert rcs.get_detached_tool_ids() == [gripper_id]
    # Even though the workpiece `.attached_to_tool` is not unset, it will not be returned here.
    assert rcs.get_attached_workpiece_ids(group) == []

    # Detach the print tool
    rcs.set_tool_detached(print_tool_id)
    rcs.set_tool_detached(gripper_id, frame=Frame.worldXY(), touch_links=["link0"])
    assert rcs.get_attached_tool_id(group) == None
    assert rcs.get_attached_workpiece_ids(group) == []

    # Attach workpiece to tool
    rcs.set_tool_attached_to_group(gripper_id, group)
    rcs.set_rigid_body_attached_to_tool(beam_id, gripper_id, attachment_frame)
    assert rcs.get_attached_workpiece_ids(group) == [beam_id]
    assert rcs.get_attached_rigid_body_ids() == [beam_id]

    # Attach workpiece to link
    rcs.set_rigid_body_attached_to_link(beam_id, "link0")
    assert rcs.get_attached_workpiece_ids(group) == []
    assert rcs.get_attached_rigid_body_ids() == [beam_id]
    assert rcs.rigid_body_states[beam_id].attachment_frame == Frame.worldXY()
    rcs.set_rigid_body_attached_to_link(beam_id, "link0", attachment_frame=attachment_frame)
    assert rcs.rigid_body_states[beam_id].attachment_frame == attachment_frame

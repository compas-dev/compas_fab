from copy import deepcopy

import pytest
from compas import IPY
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_robots import ToolModel

from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCell  # noqa: F401
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState  # noqa: F401
from compas_fab.robots import TargetMode
from compas_fab.robots import ToolState

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Tuple  # noqa: F401


@pytest.fixture
def panda():
    return RobotCellLibrary.panda(load_geometry=False)


@pytest.fixture
def ur10e_gripper_one_beam():
    return RobotCellLibrary.ur10e_gripper_one_beam(load_geometry=False)


@pytest.fixture
def abb_irb4600_40_255_gripper_one_beam():
    return RobotCellLibrary.abb_irb4600_40_255_gripper_one_beam(load_geometry=False)


@pytest.fixture
def abb_irb4600_40_255_printing_tool():
    return RobotCellLibrary.abb_irb4600_40_255_printing_tool(load_geometry=False)


@pytest.fixture
def rfl():
    return RobotCellLibrary.rfl(load_geometry=False)


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


def test_copy(ur10e_gripper_one_beam, abb_irb4600_40_255_gripper_one_beam, abb_irb4600_40_255_printing_tool, panda):
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

    test(*ur10e_gripper_one_beam)
    test(*abb_irb4600_40_255_gripper_one_beam)
    test(*abb_irb4600_40_255_printing_tool)
    test(*panda)


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
        assert robot_cell_state.sha256() == robot_cell_state_copy.sha256()

    test(*ur10e_gripper_one_beam)
    test(*abb_irb4600_40_255_gripper_one_beam)
    test(*abb_irb4600_40_255_printing_tool)


def test_state_match(ur10e_gripper_one_beam, abb_irb4600_40_255_gripper_one_beam, abb_irb4600_40_255_printing_tool):
    """Test to make sure the RobotCellState object is hashable"""

    def test(rc, rcs):
        # type: (RobotCell, RobotCellState) -> None
        rc.assert_cell_state_match(rcs)
        rcs = deepcopy(rcs)

        # Test adding extra rigid body state
        rcs.rigid_body_states["new"] = RigidBodyState(Frame.worldXY())
        with pytest.raises(ValueError):
            rc.assert_cell_state_match(rcs)
        del rcs.rigid_body_states["new"]
        rc.assert_cell_state_match(rcs)

        # Test adding extra tool state
        rcs.tool_states["new"] = ToolState(Frame.worldXY())
        with pytest.raises(ValueError):
            rc.assert_cell_state_match(rcs)
        del rcs.tool_states["new"]
        rc.assert_cell_state_match(rcs)

    test(*ur10e_gripper_one_beam)
    test(*abb_irb4600_40_255_gripper_one_beam)
    test(*abb_irb4600_40_255_printing_tool)


def test_names(ur10e_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    """Test to make sure the names are set correctly"""

    rc, rcs = ur10e_gripper_one_beam
    assert rc.root_name == rc.robot_model.root.name
    assert rc.root_name == rc.get_base_link_name()
    assert rc.get_base_link_name() == rc.get_base_link().name
    assert len(rc.get_link_names_with_collision_geometry()) == 7

    assert rc.group_names == rc.robot_semantics.group_names
    assert rc.get_end_effector_link_name() == rc.get_end_effector_link().name


def test_get_link_names(panda, rfl, ur10e_gripper_one_beam, abb_irb4600_40_255_gripper_one_beam):
    def _test(rc, rcs):
        # type: (RobotCell, RobotCellState) -> None

        all_link_names = [link.name for link in rc.robot_model.links]
        for group in rc.group_names:
            group_link_names = rc.get_link_names(group)
            assert all(link_name in all_link_names for link_name in group_link_names)

            # Get link names can return more links that that of the configurable joints
            # because some joints within the group chain may by Fixed
            # Here we can only check the joints neighboring the configurable joints
            # is a subset of all link names
            group_joints = rc.get_configurable_joints(group)
            group_joints_link_names = []
            for joint in group_joints:
                group_joints_link_names.append(str(joint.child.link))
                group_joints_link_names.append(str(joint.parent.link))
            if not set(group_joints_link_names).issubset(set(group_link_names)):
                # NOTE: The following debug code is added during the debug of issue
                # related to the non-serial chain of panda robot group "panda_hand"
                print(set(group_joints_link_names))
                print(set(group_link_names))
                assert False

    _test(*panda)
    _test(*rfl)
    _test(*ur10e_gripper_one_beam)
    _test(*abb_irb4600_40_255_gripper_one_beam)


def test_group_states(rfl):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    rc, rcs = rfl
    group_states = rc.group_states
    for group_name in group_states:
        group_state_names = rc.get_group_states_names(group_name)
        for group_state_name in group_states[group_name]:
            assert group_state_name in group_state_names
            configuration = rc.get_configuration_from_group_state(group_name, group_state_name)
            assert set(configuration.joint_names) == set(rc.get_configurable_joint_names(group_name))


def test_ensure(ur10e_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    """Test to make sure the ensure_* methods work"""

    rc, rcs = ur10e_gripper_one_beam
    rc = deepcopy(rc)

    # In this test file, the robot cell is created without geometry
    with pytest.raises(Exception):
        rc.ensure_geometry()

    # Ensure Semantics
    rc.ensure_semantics()
    rc.robot_semantics = None
    with pytest.raises(Exception):
        rc.ensure_semantics()


def test_zero_configuration(panda, rfl, ur10e_gripper_one_beam):
    """Test to make sure the zero_configuration method works"""

    def _test(rc):
        # type: (RobotCell) -> None
        # Zero Full Configuration is for all the joints
        all_configurable_joint_names = rc.robot_model.get_configurable_joint_names()
        assert all_configurable_joint_names == rc.zero_full_configuration().joint_names

        # Zero configuration is related to a planning group
        for group_name in rc.group_names:
            group_configurable_joint_names = rc.get_configurable_joint_names(group_name)
            assert group_configurable_joint_names == rc.zero_configuration(group_name).joint_names

    _test(panda[0])
    _test(rfl[0])
    _test(ur10e_gripper_one_beam[0])


def test_random_configuration(panda, rfl, ur10e_gripper_one_beam):
    """Test to make sure the random_configuration method works"""

    def _test(rc):
        # type: (RobotCell) -> None
        # Random Full Configuration is for all the joints
        all_configurable_joint_names = [j.name for j in rc.get_all_configurable_joints()]
        assert all_configurable_joint_names == rc.robot_model.random_configuration().joint_names

        # Random configuration is related to a planning group
        for group_name in rc.group_names:
            group_configurable_joint_names = rc.get_configurable_joint_names(group_name)
            assert group_configurable_joint_names == rc.random_configuration(group_name).joint_names

    _test(panda[0])
    _test(rfl[0])
    _test(ur10e_gripper_one_beam[0])


def test_full_configuration_to_group_configuration(panda, rfl, ur10e_gripper_one_beam):
    """Test to make sure the full_configuration_to_group_configuration method works"""

    def _test(rc):
        # type: (RobotCell) -> None
        # Full Configuration includes all the joints but not mimic joints
        full_configuration = rc.zero_full_configuration()

        for group in rc.group_names:
            group_configuration = rc.full_configuration_to_group_configuration(full_configuration, group)
            joint_names = rc.get_configurable_joint_names(group)
            assert set(group_configuration.joint_names) == set(joint_names)

    _test(panda[0])
    _test(rfl[0])
    _test(ur10e_gripper_one_beam[0])


def test_group_configuration_to_full_configuration(panda, rfl, ur10e_gripper_one_beam):
    """Test to make sure the full_configuration_to_group_configuration method works"""

    def _test(rc):
        # type: (RobotCell) -> None
        full_joint_names = rc.get_all_configurable_joint_names()
        for group in rc.group_names:
            group_configuration = rc.zero_configuration(group)
            full_configuration = rc.configuration_to_full_configuration(group_configuration)
            assert set(full_configuration.joint_names) == set(full_joint_names)

    _test(panda[0])
    _test(rfl[0])
    _test(ur10e_gripper_one_beam[0])


def test_attached_tool(panda, rfl, ur10e_gripper_one_beam, abb_irb4600_40_255_printing_tool):
    """Test to make sure the attached_tool method works"""

    def _test_attached_tool(rc, rcs):
        # type: (RobotCell, RobotCellState) -> None
        tool = rc.get_attached_tool(rcs, rc.main_group_name)
        assert tool is not None
        assert isinstance(tool, ToolModel)

    def _test_no_attached_tool(rc, rcs):
        # type: (RobotCell, RobotCellState) -> None
        tool = rc.get_attached_tool(rcs, rc.main_group_name)
        assert tool is None

    _test_no_attached_tool(*panda)
    _test_no_attached_tool(*rfl)
    _test_attached_tool(*ur10e_gripper_one_beam)
    _test_attached_tool(*abb_irb4600_40_255_printing_tool)


def test_attached_workpiece(panda, rfl, ur10e_gripper_one_beam, abb_irb4600_40_255_printing_tool):
    """Test to make sure the attached_tool method works"""

    def _test_attached_workpiece(rc, rcs):
        # type: (RobotCell, RobotCellState) -> None
        workpieces = rc.get_attached_workpieces(rcs, rc.main_group_name)
        assert workpieces != []
        for workpiece in workpieces:
            assert isinstance(workpiece, RigidBody)

    def _test_no_attached_workpiece(rc, rcs):
        # type: (RobotCell, RobotCellState) -> None
        workpieces = rc.get_attached_workpieces(rcs, rc.main_group_name)
        assert workpieces == []

    _test_no_attached_workpiece(*panda)
    _test_no_attached_workpiece(*rfl)
    _test_no_attached_workpiece(*abb_irb4600_40_255_printing_tool)
    _test_attached_workpiece(*ur10e_gripper_one_beam)


# TODO: Add test for get_attached_rigid_bodies


def test_transformations(ur10e_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    """Test to make sure the transformations method works"""

    rc, rcs = ur10e_gripper_one_beam
    tool_id = rc.tool_ids[0]
    workpiece_id = rc.rigid_body_ids[0]

    # The general FK formula : t_wcf_ocf = t_wcf_pcf * t_pcf_t0cf * t_t0cf_tcf * t_tcf_ocf

    # Define the transformations
    tool_attachment_frame = Frame([1, 2, 3], [0.5, 0.5, 0], [0, 1, 0])
    t_pcf_t0cf = Transformation.from_frame(tool_attachment_frame)
    rcs.tool_states[tool_id].attachment_frame = tool_attachment_frame

    tool_frame = rc.tool_models[tool_id].frame
    t_t0cf_tcf = Transformation.from_frame(tool_frame)

    work_attachment_frame = Frame([2, 3, 4], [-0.5, 0.5, 0], [0, 1, 0])
    t_tcf_ocf = Transformation.from_frame(work_attachment_frame)
    rcs.rigid_body_states[workpiece_id].attachment_frame = work_attachment_frame

    # Test the transformation functions
    t_pcf_tcf = rc.t_pcf_tcf(rcs, tool_id)
    assert t_pcf_tcf == t_pcf_t0cf * t_t0cf_tcf
    t_pcf_ocf = rc.t_pcf_ocf(rcs, workpiece_id)
    assert t_pcf_ocf == t_pcf_t0cf * t_t0cf_tcf * t_tcf_ocf

    # Test the functions for transforming Tool targets
    tcf_frame = Frame([4, 5, 6], [0.5, 0.5, 0], [0, 1, 0])
    t_wcf_tcf = Transformation.from_frame(tcf_frame)
    pcf_frame = rc.from_tcf_to_pcf(rcs, [tcf_frame], tool_id)[0]
    t_wcf_pcf = Transformation.from_frame(pcf_frame)
    assert t_wcf_pcf == t_wcf_tcf * (t_pcf_t0cf * t_t0cf_tcf).inverse()
    assert rc.from_pcf_to_tcf(rcs, [pcf_frame], tool_id)[0] == tcf_frame
    assert rc.target_frames_to_pcf(rcs, tcf_frame, TargetMode.TOOL, rc.main_group_name) == pcf_frame
    assert rc.target_frames_to_pcf(rcs, [tcf_frame], TargetMode.TOOL, rc.main_group_name) == [pcf_frame]
    assert rc.pcf_to_target_frames(rcs, pcf_frame, TargetMode.TOOL, rc.main_group_name) == tcf_frame
    assert rc.pcf_to_target_frames(rcs, [pcf_frame], TargetMode.TOOL, rc.main_group_name) == [tcf_frame]

    # Test the functions for transforming Workpiece targets
    ocf_frame = Frame([5, 6, 7], [-0.5, 0.5, 0], [0, 1, 0])
    t_wcf_ocf = Transformation.from_frame(ocf_frame)
    pcf_frame = rc.from_ocf_to_pcf(rcs, [ocf_frame], workpiece_id)[0]
    t_wcf_pcf = Transformation.from_frame(pcf_frame)
    assert t_wcf_pcf == t_wcf_ocf * (t_pcf_t0cf * t_t0cf_tcf * t_tcf_ocf).inverse()
    assert rc.from_pcf_to_ocf(rcs, [pcf_frame], workpiece_id)[0] == ocf_frame
    assert rc.target_frames_to_pcf(rcs, ocf_frame, TargetMode.WORKPIECE, rc.main_group_name) == pcf_frame
    assert rc.target_frames_to_pcf(rcs, [ocf_frame], TargetMode.WORKPIECE, rc.main_group_name) == [pcf_frame]
    assert rc.pcf_to_target_frames(rcs, pcf_frame, TargetMode.WORKPIECE, rc.main_group_name) == ocf_frame
    assert rc.pcf_to_target_frames(rcs, [pcf_frame], TargetMode.WORKPIECE, rc.main_group_name) == [ocf_frame]

    # Final test for no transform with a robot target
    assert rc.target_frames_to_pcf(rcs, tcf_frame, TargetMode.ROBOT, rc.main_group_name) == tcf_frame
    assert rc.pcf_to_target_frames(rcs, tcf_frame, TargetMode.ROBOT, rc.main_group_name) == tcf_frame


def test_compute_attach_objects_frames(ur10e_gripper_one_beam):
    # type: (Tuple[RobotCell, RobotCellState]) -> None
    """Test to make sure the compute_attach_objects_frames method works"""

    rc, rcs = ur10e_gripper_one_beam
    tool_id = rc.tool_ids[0]
    workpiece_id = rc.rigid_body_ids[0]

    # Define a non zero robot_base_frame
    rcs.robot_base_frame = Frame([0, 1, 2], [0.5, 0.5, 0], [0, 1, 0])

    # Define the tool attachment
    tool_attachment_frame = Frame([1, 2, 3], [0.5, -0.5, 0], [0, 1, 0])
    rcs.tool_states[tool_id].attachment_frame = tool_attachment_frame
    rcs.tool_states[tool_id].attached_to_group = rc.main_group_name

    # Define the workpiece attachment
    work_attachment_frame = Frame([2, 3, 4], [-0.5, 0.5, 0], [0, 1, 0])
    rcs.rigid_body_states[workpiece_id].attachment_frame = work_attachment_frame

    # Use the transformation functions for the test
    t_pcf_tcf = rc.t_pcf_tcf(rcs, tool_id)
    t_pcf_ocf = rc.t_pcf_ocf(rcs, workpiece_id)

    # Define the expected frames
    t_wcf_rcf = Transformation.from_frame(rcs.robot_base_frame)
    ee_link_name = rc.get_end_effector_link_name(group=rc.main_group_name)
    t_rcf_pcf = Transformation.from_frame(rc.robot_model.forward_kinematics(rcs.robot_configuration, ee_link_name))
    t_wcf_tcf = t_wcf_rcf * t_rcf_pcf * t_pcf_tcf
    t_wcf_ocf = t_wcf_rcf * t_rcf_pcf * t_pcf_ocf

    expected_tool_frame = Frame.from_transformation(t_wcf_tcf)
    expected_workpiece_frame = Frame.from_transformation(t_wcf_ocf)

    # Test the function
    computed_rcs = rc.compute_attach_objects_frames(rcs)
    assert computed_rcs.tool_states[tool_id].frame == expected_tool_frame
    assert computed_rcs.rigid_body_states[workpiece_id].frame == expected_workpiece_frame

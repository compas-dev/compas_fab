import pytest
import compas

if not compas.IPY:
    from compas_fab.backends import PyBulletClient
    from compas_fab.backends import PyBulletPlanner

from compas_fab.backends.pybullet import PyBulletClient
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_fab.backends.exceptions import CollisionCheckError

from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotLibrary
from compas_fab.robots import RobotCellState
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import TargetMode
from compas_robots import Configuration

from compas.geometry import Frame


@pytest.fixture
def pybullet_client():
    with PyBulletClient(connection_type="direct") as client:
        yield client


@pytest.fixture
def planner_with_test_cell(pybullet_client):
    planner = PyBulletPlanner(pybullet_client)

    # Setup the robot cell
    robot_cell, robot_cell_state = RobotCellLibrary.ur10e_gripper_one_beam()
    planner.set_robot_cell(robot_cell, robot_cell_state)

    # Modify the beam attachment frame slightly so that the TCP is not the same as OCF
    robot_cell_state.rigid_body_states["beam"].attachment_frame = Frame(
        [0.1, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
    )

    return planner, robot_cell, robot_cell_state


def test_inverse_kinematics_frame_target_target_modes(planner_with_test_cell):
    planner, robot_cell, robot_cell_state = planner_with_test_cell

    group = robot_cell.robot.main_group_name
    options = {
        "check_collision": False,
    }

    # Test with a frame target
    target_frame = Frame([0.5, 0.5, 0.5], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    target = FrameTarget(target_frame, TargetMode.ROBOT)
    result_robot = planner.inverse_kinematics(target, robot_cell_state, options=options)
    assert isinstance(result_robot, Configuration)

    target = FrameTarget(target_frame, TargetMode.TOOL)
    result_tool = planner.inverse_kinematics(target, robot_cell_state, options=options)
    assert isinstance(result_tool, Configuration)

    target = FrameTarget(target_frame, TargetMode.WORKPIECE)
    result_workpiece = planner.inverse_kinematics(target, robot_cell_state, options=options)
    assert isinstance(result_workpiece, Configuration)

    # Assert that the results are different
    assert not result_robot.close_to(result_tool)
    assert not result_robot.close_to(result_workpiece)
    assert not result_tool.close_to(result_workpiece)

    #  Assert that when converting the results to frames, they match up with the original target
    fk_options = {"link": robot_cell.robot.get_end_effector_link_name(group)}
    robot_cell_state.robot_configuration = result_robot
    fk_frame_robot = planner.forward_kinematics(robot_cell_state, group, options=fk_options)
    assert fk_frame_robot.__eq__(target_frame, tol=1e-3)

    robot_cell_state.robot_configuration = result_tool
    fk_frame_tool = planner.forward_kinematics(robot_cell_state, group, options=fk_options)
    fk_frame_tcf = planner.from_pcf_to_tcf([fk_frame_tool], "gripper")[0]
    assert fk_frame_tcf.__eq__(target_frame, tol=1e-3)

    robot_cell_state.robot_configuration = result_workpiece
    fk_frame_workpiece = planner.forward_kinematics(robot_cell_state, group, options=fk_options)
    fk_frame_ocf = planner.from_pcf_to_ocf([fk_frame_workpiece], "beam")[0]
    assert fk_frame_ocf.__eq__(target_frame, tol=1e-3)


def test_inverse_kinematics_target_mode_validation(planner_with_test_cell):
    """Test that the inverse kinematics function correctly handles the target modes.

    The inverse kinematics function should raise an error if the target mode is not possible

    Notes
    -----
    Subsequent motion planning functions depended on the
    correctness of the IK function handling target modes.
    """
    planner, robot_cell, robot_cell_state = planner_with_test_cell

    target_frame = Frame([0.5, 0.5, 0.5], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    options = {
        "check_collision": False,
    }

    # Test without a workpiece attached
    robot_cell_state.rigid_body_states["beam"].attached_to_tool = False
    robot_cell_state.rigid_body_states["beam"].frame = Frame.worldXY()
    planner.set_robot_cell_state(robot_cell_state)

    # WORKPIECE target mode should not be possible
    target = FrameTarget(target_frame, TargetMode.WORKPIECE)
    with pytest.raises(ValueError):
        planner.inverse_kinematics(target, robot_cell_state, options=options)
    # TOOL target mode should be possible
    target = FrameTarget(target_frame, TargetMode.TOOL)
    planner.inverse_kinematics(target, robot_cell_state, options=options)
    # ROBOT target mode should be possible
    target = FrameTarget(target_frame, TargetMode.ROBOT)
    planner.inverse_kinematics(target, robot_cell_state, options=options)

    #  -----------------------------

    # Test without a tool attached
    robot_cell_state.tool_states["gripper"].attached_to_group = False
    robot_cell_state.tool_states["gripper"].frame = Frame.worldXY()
    planner.set_robot_cell_state(robot_cell_state)

    # WORKPIECE target mode should not be possible
    target = FrameTarget(target_frame, TargetMode.WORKPIECE)
    with pytest.raises(ValueError):
        planner.inverse_kinematics(target, robot_cell_state, options=options)
    # TOOL target mode should not be possible
    target = FrameTarget(target_frame, TargetMode.TOOL)
    with pytest.raises(ValueError):
        planner.inverse_kinematics(target, robot_cell_state)
    # ROBOT target mode should be possible
    target = FrameTarget(target_frame, TargetMode.ROBOT)
    planner.inverse_kinematics(target, robot_cell_state, options=options)


def test_iter_inverse_kinematics_frame_target(planner_with_test_cell):
    planner, robot_cell, robot_cell_state = planner_with_test_cell

    # Basic test to produce results

    options = {
        "max_results": 1,
        "high_accuracy": True,
        "check_collision": True,
        "verbose": False,
    }

    # This target is reachable and collision free
    target = FrameTarget(Frame([0.5, 0.5, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), TargetMode.ROBOT)

    # Assert that if someone forgot to provide a group or an invalid one, it will raise an error
    generator = planner.iter_inverse_kinematics_frame_target(target, robot_cell_state, options)
    with pytest.raises(TypeError):
        next(generator)
    generator = planner.iter_inverse_kinematics_frame_target(target, robot_cell_state, "non-exist-group", options)
    with pytest.raises(ValueError):
        next(generator)

    group = robot_cell.robot.main_group_name
    generator = planner.iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
    result = next(generator)  # type: Configuration

    # Check that there is a result
    assert isinstance(result, Configuration)
    assert len(result.joint_values) == 6

    # Test with a target that is not reachable / very far away
    target = FrameTarget(Frame([10.0, 10.0, 10.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), TargetMode.ROBOT)
    generator = planner.iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
    with pytest.raises(InverseKinematicsError):
        next(generator)

    # # Test with a target that will collide with the floor
    options.update({"check_collision": True, "max_results": 1})
    target = FrameTarget(Frame([0.5, 0.5, -0.2], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), TargetMode.ROBOT)
    generator = planner.iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
    # # When the option max_result == 1 , it will return CollisionCheckError when target is still reachable
    with pytest.raises(CollisionCheckError):
        next(generator)

    options.update({"check_collision": True, "max_results": 10})
    generator = planner.iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
    # # When the option max_result > 1 , it will return InverseKinematicsError after the search
    with pytest.raises(InverseKinematicsError):
        next(generator)

    # Test that when collision check is disabled, it will return a result
    options.update({"check_collision": False, "max_results": 10})
    generator = planner.iter_inverse_kinematics_frame_target(target, robot_cell_state, group, options)
    result = next(generator)  # type: Configuration
    assert isinstance(result, Configuration)


def test_iter_inverse_kinematics_point_axis_target(planner_with_test_cell):
    planner, robot_cell, robot_cell_state = planner_with_test_cell

    # Basic test to produce results

    options = {
        "check_collision": True,
        "verbose": False,
    }

    # This target is reachable and collision free
    target = PointAxisTarget([0.5, 0.5, 1.0], [1.0, 0.0, 0.0], TargetMode.ROBOT)

    # Assert that if someone forgot to provide a group or an invalid one, it will raise an error
    generator = planner.iter_inverse_kinematics_point_axis_target(target, robot_cell_state, options)
    with pytest.raises(TypeError):
        next(generator)
    generator = planner.iter_inverse_kinematics_point_axis_target(target, robot_cell_state, "non-exist-group", options)
    with pytest.raises(ValueError):
        next(generator)

    group = robot_cell.robot.main_group_name
    generator = planner.iter_inverse_kinematics_point_axis_target(target, robot_cell_state, group, options)
    result = next(generator)  # type: Configuration

    # Check that there is a result
    assert isinstance(result, Configuration)
    assert len(result.joint_values) == 6

    # Test with a target that is not reachable / very far away
    target = PointAxisTarget([10, 10, 10], [1.0, 0.0, 0.0], TargetMode.ROBOT)
    generator = planner.iter_inverse_kinematics_point_axis_target(target, robot_cell_state, group, options)
    with pytest.raises(InverseKinematicsError):
        next(generator)

    # # Test with a target that will collide with the floor
    options.update({"check_collision": True, "max_random_restart": 10})
    target = PointAxisTarget([0.5, 0.5, -0.2], [1.0, 0.0, 0.0], TargetMode.ROBOT)
    generator = planner.iter_inverse_kinematics_point_axis_target(target, robot_cell_state, group, options)
    # # When the option max_result == 1 , it will return CollisionCheckError when target is still reachable
    with pytest.raises(InverseKinematicsError):
        next(generator)

    # Test that when collision check is disabled, it will return a result
    options.update({"check_collision": False, "max_random_restart": 10})
    generator = planner.iter_inverse_kinematics_point_axis_target(target, robot_cell_state, group, options)
    result = next(generator)  # type: Configuration
    assert isinstance(result, Configuration)

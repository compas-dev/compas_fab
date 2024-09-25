from copy import deepcopy

import pytest
from compas import IPY

if not IPY:
    from compas_fab.backends import PyBulletClient
    from compas_fab.backends import PyBulletPlanner

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import axis_and_angle_from_matrix
from compas.geometry import matrix_from_frame
from compas_robots import Configuration
from compas_robots.model import Joint

from compas_fab.backends import CollisionCheckError
from compas_fab.backends import InverseKinematicsError
from compas_fab.backends import PlanningGroupNotExistsError
from compas_fab.backends import PlanningGroupNotSupported
from compas_fab.backends import TargetModeMismatchError
from compas_fab.robots import FrameTarget
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import Robot  # noqa: F401
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary
from compas_fab.robots import TargetMode


@pytest.fixture
def pybullet_client():
    with PyBulletClient(connection_type="direct") as client:
        yield client


def compare_frames(frame1, frame2, tolerance_position, tolerance_orientation):
    # type: (Frame, Frame, float, float) -> bool
    """Helper function to compare two frames in terms of position and orientation tolerance."""
    delta_frame = frame1.to_local_coordinates(frame2)  # type: Frame
    if Vector(*delta_frame.point).length > tolerance_position:
        return False
    axis, angle = axis_and_angle_from_matrix(matrix_from_frame(delta_frame))
    if angle > tolerance_orientation:
        return False
    return True


def compare_configuration(config1, config2, tolerance_linear, tolerance_angular):
    # type: (Configuration, Configuration, float, float) -> bool
    """Helper function to compare two configurations in terms of joint values."""

    if config1.joint_names and config2.joint_names:
        if set(config1.joint_names) != set(config2.joint_names):
            raise ValueError("Configurations have different joint names.")
        other_value_by_name = dict(zip(config2.joint_names, config2.joint_values))
        sorted_other_values = [other_value_by_name[name] for name in config1.joint_names]
        value_pairs = zip(config1.joint_values, sorted_other_values)
    else:
        if len(config1.joint_values) != len(config2.joint_values):
            raise ValueError("Can't compare configurations with different lengths of joint_values.")
        value_pairs = zip(config1.joint_values, config2.joint_values)

    for i, (v1, v2) in enumerate(value_pairs):
        diff = v1 - v2
        if config1.joint_types[i] in [Joint.REVOLUTE, Joint.CONTINUOUS]:
            if abs(diff) > tolerance_angular:
                return False
        else:
            if abs(diff) > tolerance_linear:
                return False
    return True


######################################################
# Testing FK function with different robots
######################################################


def compare_planner_fk_model_fk(robot, pybullet_client, known_zero_frame):
    # type: (Robot, PyBulletClient, Frame) -> None
    """A test of forward kinematics using the zero configuration of the robot."""
    planner = PyBulletPlanner(pybullet_client)
    planner.set_robot_cell(RobotCell(robot))

    planning_group = robot.main_group_name
    end_effector_link = robot.get_end_effector_link_name(planning_group)

    robot_cell_state = RobotCellState.from_robot_configuration(robot, robot.zero_configuration())
    planner_fk_result = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT, group=planning_group)
    print(planner_fk_result)
    model_fk_result = robot.model.forward_kinematics(robot.zero_configuration(), end_effector_link)
    print(model_fk_result)

    assert compare_frames(planner_fk_result, model_fk_result, 1e-3, 1e-3)
    assert compare_frames(planner_fk_result, known_zero_frame, 1e-3, 1e-3)


def test_fk_ur5(pybullet_client):
    robot = RobotLibrary.ur5(load_geometry=True)
    known_zero_frame = Frame(
        point=Point(x=0.8172500133514404, y=0.19144999980926514, z=-0.005491000134497881),
        xaxis=Vector(x=-1.0, y=0.0, z=0.0),
        yaxis=Vector(x=0.0, y=0.0, z=1.0),
    )
    compare_planner_fk_model_fk(robot, pybullet_client, known_zero_frame)


def test_fk_abb_irb4600_40_255(pybullet_client):
    robot = RobotLibrary.abb_irb4600_40_255(load_geometry=True)
    known_zero_frame = Frame(
        point=Point(x=1.58, y=0.0, z=1.765),
        xaxis=Vector(x=0.0, y=0.0, z=-1.0),
        yaxis=Vector(x=0.0, y=1.0, z=0.0),
    )
    compare_planner_fk_model_fk(robot, pybullet_client, known_zero_frame)


def test_fk_ur10e(pybullet_client):
    robot = RobotLibrary.ur10e(load_geometry=True)
    known_zero_frame = Frame(
        point=Point(x=1.18425, y=0.2907, z=0.0608),
        xaxis=Vector(x=-1.0, y=0.0, z=-0.0),
        yaxis=Vector(x=0.0, y=0.0, z=1.0),
    )
    compare_planner_fk_model_fk(robot, pybullet_client, known_zero_frame)


def test_fk_panda(pybullet_client):
    robot = RobotLibrary.panda(load_geometry=True)
    known_zero_frame = Frame(
        point=Point(x=0.256, y=-0.000, z=0.643),
        xaxis=Vector(x=-0.000, y=0.707, z=-0.707),
        yaxis=Vector(x=-0.000, y=-0.707, z=-0.707),
    )
    compare_planner_fk_model_fk(robot, pybullet_client, known_zero_frame)


######################################################
# Testing the IK-FK agreement for the PyBullet backend
######################################################


def ik_fk_agreement(robot, pybullet_client, ik_target_frames):
    """Helper function to test the IK-FK agreement for the PyBullet backend

    The function takes a robot and a list of target frames for IK and FK queries.
    IK is first performed by calling iter_inverse_kinematics() and later FK is performed by calling forward_kinematics().
    The closeness of the starting and ending frames are checked to ensure the IK-FK agreement.

    """
    # These options are set to ensure that the IK solver converges to a high accuracy
    # Target.tolerance_orientation is set to 1e-5 meters to be larger than the tolerance used for comparison (1e-4 meters)
    tolerance_position = 1e-5
    tolerance_orientation = 1e-2
    ik_options = {
        "max_descend_iterations": 50,
        "return_full_configuration": True,
    }

    planner = PyBulletPlanner(pybullet_client)
    planner.set_robot_cell(RobotCell(robot))
    planning_group = robot.main_group_name

    for ik_target_frame in ik_target_frames:
        # IK Query to the planner (Frame to Configuration)
        ik_target_frame = deepcopy(ik_target_frame)  # type: FrameTarget
        try:

            ik_result = next(
                planner.iter_inverse_kinematics(
                    FrameTarget(
                        ik_target_frame,
                        target_mode=TargetMode.ROBOT,
                        tolerance_position=tolerance_position,
                        tolerance_orientation=tolerance_orientation,
                    ),
                    RobotCellState.from_robot_configuration(robot),
                    group=planning_group,
                    options=ik_options,
                )
            )
        except StopIteration:
            assert False, f"No IK Solution found for frame {ik_target_frame}"

        # FK Query to the planner (Configuration to Frame)
        robot_cell_state = RobotCellState.from_robot_configuration(robot, ik_result)
        fk_result = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT, group=planning_group)

        # Compare the frames
        assert compare_frames(ik_target_frame, fk_result, tolerance_position, tolerance_orientation)


def test_ik_fk_agreement_ur5(pybullet_client):
    robot = RobotLibrary.ur5(load_geometry=True)

    ik_center_frame = Frame(
        Point(x=0.4, y=0.1, z=0.3),
        Vector(x=-1.0, y=0.0, z=0.0),
        Vector(x=0.0, y=0.0, z=1.0),
    )

    ik_target_frames = []
    ik_target_frames.append(ik_center_frame)
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.01, -0.01, -0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.01, 0.0)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.0, 0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.1, 0.1, 0.2)))
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.1, -0.1, 0.0)))

    ik_fk_agreement(robot, pybullet_client, ik_target_frames)


def test_ik_fk_agreement_ur10e(pybullet_client):
    robot = RobotLibrary.ur10e(load_geometry=True)

    ik_center_frame = Frame(
        Point(x=0.4, y=0.1, z=0.3),
        Vector(x=-1.0, y=0.0, z=0.0),
        Vector(x=0.0, y=0.0, z=1.0),
    )

    ik_target_frames = []
    ik_target_frames.append(ik_center_frame)
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.01, -0.01, -0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.01, 0.0)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.0, 0.01)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.1, 0.1, 0.2)))
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.1, -0.1, 0.0)))

    ik_fk_agreement(robot, pybullet_client, ik_target_frames)


def test_ik_fk_agreement_abb_irb4600_40_255(pybullet_client):
    robot = RobotLibrary.abb_irb4600_40_255(load_geometry=True)

    ik_center_frame = Frame(
        Point(x=1.0, y=0.3, z=1.3),
        Vector(x=-1.0, y=0.0, z=0.0),
        Vector(x=0.0, y=0.0, z=1.0),
    )

    ik_target_frames = []
    ik_target_frames.append(ik_center_frame)
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.1, -0.1, -0.1)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.1, 0.0)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.0, 0.1)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.1, 0.1, 0.2)))
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.3, -0.1, -0.3)))

    ik_fk_agreement(robot, pybullet_client, ik_target_frames)


def test_ik_fk_agreement_panda(pybullet_client):
    # The panda robot has mimic joints for testing purposes
    robot = RobotLibrary.panda(load_geometry=True)

    ik_center_frame = Frame(
        point=Point(x=0.2, y=-0.0, z=0.6),
        xaxis=Vector(x=0.0, y=1.0, z=-0.0),
        yaxis=Vector(x=0.0, y=0.0, z=-1.0),
    )

    ik_target_frames = []
    ik_target_frames.append(ik_center_frame)
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.1, -0.1, -0.1)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.1, 0.0)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.0, 0.0, 0.1)))
    ik_target_frames.append(ik_center_frame.translated(Vector(0.1, 0.1, -0.2)))
    ik_target_frames.append(ik_center_frame.translated(Vector(-0.05, -0.05, -0.03)))

    ik_fk_agreement(robot, pybullet_client, ik_target_frames)


##################################################
# Testing IK out of reach for the PyBullet backend
##################################################


def test_ik_out_of_reach_ur5(pybullet_client):
    robot = RobotLibrary.ur5(load_geometry=True)

    ik_target_frames = []

    ik_target_frames.append(
        Frame(
            Point(x=2.4, y=0.1, z=0.3),
            Vector(x=-1.0, y=0.0, z=0.0),
            Vector(x=0.0, y=0.0, z=1.0),
        )
    )
    ik_target_frames.append(
        Frame(
            Point(x=0.4, y=1.5, z=0.3),
            Vector(x=-1.0, y=0.0, z=0.0),
            Vector(x=0.0, y=0.0, z=1.0),
        )
    )

    # max_descend_iterations is set to 20 to reduce the number of iterations, faster testing time.
    ik_options = {
        "max_descend_iterations": 20,
    }

    planner = PyBulletPlanner(pybullet_client)
    planner.set_robot_cell(RobotCell(robot))
    planning_group = robot.main_group_name
    for ik_target_frame in ik_target_frames:
        # IK Query to the planner (Frame to Configuration)
        try:
            # Note: The inverse_kinematics method returns a generator
            planner.inverse_kinematics(
                FrameTarget(ik_target_frame, target_mode=TargetMode.ROBOT),
                RobotCellState.from_robot_configuration(robot),
                group=planning_group,
                options=ik_options,
            )

            # An error should be thrown here because the IK target is out of reach
            assert False, f"IK Solution found when there should be none: frame {ik_target_frame}"
        except InverseKinematicsError:
            continue


def test_ik_return_full_configuration(pybullet_client):
    robot = RobotLibrary.panda(load_geometry=True)

    ik_target_frame = Frame(
        point=Point(x=0.2, y=-0.0, z=0.6),
        xaxis=Vector(x=0.0, y=1.0, z=-0.0),
        yaxis=Vector(x=0.0, y=0.0, z=-1.0),
    )
    target = FrameTarget(ik_target_frame, TargetMode.ROBOT)

    planner = PyBulletPlanner(pybullet_client)
    planner.set_robot_cell(RobotCell(robot))
    robot_cell_state = RobotCellState.from_robot_configuration(robot, robot.zero_configuration())
    config = planner.inverse_kinematics(target, robot_cell_state)
    assert "panda_finger_joint1" not in config.joint_names
    assert len(config.joint_names) == 7

    ik_options = {"return_full_configuration": True}
    config = planner.inverse_kinematics(target, robot_cell_state, options=ik_options)
    assert "panda_finger_joint1" in config.joint_names
    assert len(config.joint_names) == 8


def test_ik_modified_only_joints_in_group(pybullet_client):
    # Test the IK solver see if it moved only the joints in the group
    robot = RobotLibrary.panda(load_geometry=True)

    # Create a custom group with joint 1 locked, `panda_joint1` and `panda_link0` are not included.
    # This is not supported because the starting part of the chain is not included
    robot.semantics.groups["locked_j1"] = {
        "links": [
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
            "panda_link7",
            "panda_link8",
            "panda_hand",
            "panda_hand_tcp",
        ],
        "joints": [
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_hand_joint",
            "panda_hand_tcp_joint",
        ],
    }

    # The following group is supported because only the right side of the chain is cut short
    robot.semantics.groups["locked_j7"] = {
        "links": [
            "panda_link0",
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
        ],
        "joints": [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
        ],
    }
    # The following group does not work because the links bridged by the joints are not included
    robot.semantics.groups["locked_j7_further"] = {
        "links": [
            "panda_link0",
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
            "panda_link7",
            "panda_link8",
            "panda_hand",
            "panda_hand_tcp",
        ],
        "joints": [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
        ],
    }

    ik_target_frame = Frame(
        point=Point(x=0.22, y=0.11, z=0.6),
        xaxis=Vector(x=0.11, y=1.0, z=-0.0),
        yaxis=Vector(x=0.11, y=0.0, z=-1.0),
    )
    target = FrameTarget(ik_target_frame, TargetMode.ROBOT)
    options = {"return_full_configuration": True}

    planner = PyBulletPlanner(pybullet_client)
    planner.set_robot_cell(RobotCell(robot))
    initial_configuration = robot.zero_configuration()
    initial_configuration["panda_joint1"] = 0.123
    initial_configuration["panda_joint7"] = 0.123
    print(initial_configuration)
    robot_cell_state = RobotCellState.from_robot_configuration(robot, initial_configuration)

    # The main planning group should work as expected
    config = planner.inverse_kinematics(target, robot_cell_state, options=options)
    print(config)

    # This group will raise PlanningGroupNotSupported because the group is not supported
    try:
        config = planner.inverse_kinematics(target, robot_cell_state, "locked_j1", options=options)
        assert False, "Should raise NotImplementedError"
    except PlanningGroupNotSupported:
        pass

    # This should work
    config = planner.inverse_kinematics(target, robot_cell_state, "locked_j7", options=options)
    assert config["panda_joint7"] == initial_configuration["panda_joint7"]

    # This should raise PlanningGroupNotSupported
    try:
        config = planner.inverse_kinematics(target, robot_cell_state, "locked_j7_further", options=options)
        assert False, "Should raise NotImplementedError"
    except PlanningGroupNotSupported:
        pass


def test_ik_fk_with_wrong_group_name_raises_exception(pybullet_client):
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()
    ik_target_frame = Frame(
        point=Point(x=0.22, y=0.11, z=0.6),
        xaxis=Vector(x=0.11, y=1.0, z=-0.0),
        yaxis=Vector(x=0.11, y=0.0, z=-1.0),
    )
    target = FrameTarget(ik_target_frame, TargetMode.ROBOT)

    planner = PyBulletPlanner(pybullet_client)
    planner.set_robot_cell(robot_cell, robot_cell_state)
    non_existent_group = "non_existent_group"
    with pytest.raises(PlanningGroupNotExistsError):
        planner.inverse_kinematics(target, robot_cell_state, group=non_existent_group)
    with pytest.raises(PlanningGroupNotExistsError):
        planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT, group=non_existent_group)

    # Planning functions (e.g. plan_motion) rely on these IK and FK functions so they should raise
    # the same exception


##################################################
# Testing IK with Targets
##################################################


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


def test_frame_target_ik_tolerance(planner_with_test_cell):
    """Test to make sure that PyBullet IK solver respects the target tolerance.
    The test checks that the IK solver respects the target tolerance for both position and orientation.
    Moreover, it ensures that the result does not overachieve the target tolerance.
    This makes sure that the IK solver is not spending more time than necessary to find a solution.
    """
    planner, robot_cell, robot_cell_state = planner_with_test_cell
    group = robot_cell.robot.main_group_name
    result_cell_state = robot_cell_state.copy()  # type: RobotCellState

    target_frame = Frame([0.5, 0.5, 0.5], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    options = {
        "check_collision": False,
    }

    def test_planning_tolerance(tolerance_position, tolerance_orientation):
        # Test with a frame target with a different tolerance and we also check that the result does not over achieve
        target = FrameTarget(
            target_frame,
            TargetMode.ROBOT,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )
        result = planner.inverse_kinematics(target, robot_cell_state, options=options)
        result_cell_state.robot_configuration = result
        result_frame = planner.forward_kinematics(result_cell_state, TargetMode.ROBOT, group=group)
        assert isinstance(result, Configuration)
        assert compare_frames(target_frame, result_frame, tolerance_position, tolerance_orientation)
        # The result should not be 2 orders of magnitude better than the target tolerance
        assert not compare_frames(target_frame, result_frame, tolerance_position * 1e-2, tolerance_orientation * 1e-2)

    # Test with unspecified tolerance (default position tolerance is 1e-3, equivalent to 1 mm)
    test_planning_tolerance(
        PyBulletPlanner.DEFAULT_TARGET_TOLERANCE_POSITION,
        PyBulletPlanner.DEFAULT_TARGET_TOLERANCE_ORIENTATION,
    )

    # Test with higher tolerance
    test_planning_tolerance(1e-4, 1e-4)

    # Test with even higher tolerance (1e-6 is equivalent to 1 micron)
    test_planning_tolerance(1e-6, 1e-6)


def test_frame_target_ik_with_tools_and_workpieces(planner_with_test_cell):
    planner, robot_cell, robot_cell_state = planner_with_test_cell

    # Testing conditions
    group = robot_cell.robot.main_group_name
    target_frame = Frame([0.5, 0.5, 0.5], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    options = {"check_collision": False}
    tolerance_position = 1e-3
    tolerance_orientation = 1e-3

    def ik_fk_comparision(target_mode):
        # Construct the IK target
        target = FrameTarget(
            target_frame,
            target_mode=target_mode,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation,
        )
        # Plan IK to get configuration
        configuration = planner.inverse_kinematics(target, robot_cell_state, group=group, options=options)
        assert isinstance(configuration, Configuration)
        # Construct new cell state with the planned configuration
        resulting_cell_state = robot_cell_state.copy()
        resulting_cell_state.robot_configuration = configuration
        fk_frame_robot = planner.forward_kinematics(resulting_cell_state, target_mode, group=group)
        assert compare_frames(fk_frame_robot, target_frame, tolerance_position, tolerance_orientation)
        return configuration

    configuration_robot_mode = ik_fk_comparision(TargetMode.ROBOT)
    configuration_tool_mode = ik_fk_comparision(TargetMode.TOOL)
    configuration_workpiece_mode = ik_fk_comparision(TargetMode.WORKPIECE)

    # Assert that the results are different
    assert not configuration_robot_mode.close_to(configuration_tool_mode)
    assert not configuration_tool_mode.close_to(configuration_workpiece_mode)
    assert not configuration_workpiece_mode.close_to(configuration_robot_mode)


def test_ik_fk_target_mode_mismatch_with_cell_state_raises_exception(planner_with_test_cell):
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

    # =================================
    # Test without a workpiece attached
    # =================================

    robot_cell_state.rigid_body_states["beam"].attached_to_tool = False
    robot_cell_state.rigid_body_states["beam"].frame = Frame.worldXY()
    planner.set_robot_cell_state(robot_cell_state)

    # WORKPIECE target mode should not be possible
    target = FrameTarget(target_frame, TargetMode.WORKPIECE)
    with pytest.raises(TargetModeMismatchError):
        planner.inverse_kinematics(target, robot_cell_state, options=options)
    with pytest.raises(TargetModeMismatchError):
        planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)

    # TOOL target mode should be possible
    target = FrameTarget(target_frame, TargetMode.TOOL)
    planner.inverse_kinematics(target, robot_cell_state, options=options)
    planner.forward_kinematics(robot_cell_state, TargetMode.TOOL)
    # ROBOT target mode should be possible
    target = FrameTarget(target_frame, TargetMode.ROBOT)
    planner.inverse_kinematics(target, robot_cell_state, options=options)
    planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)

    # =============================
    # Test without a tool attached
    # =============================

    robot_cell_state.tool_states["gripper"].attached_to_group = False
    robot_cell_state.tool_states["gripper"].frame = Frame.worldXY()
    planner.set_robot_cell_state(robot_cell_state)

    # WORKPIECE target mode should not be possible
    target = FrameTarget(target_frame, TargetMode.WORKPIECE)
    with pytest.raises(TargetModeMismatchError):
        planner.inverse_kinematics(target, robot_cell_state, options=options)
    with pytest.raises(TargetModeMismatchError):
        planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)
    # TOOL target mode should not be possible
    target = FrameTarget(target_frame, TargetMode.TOOL)
    with pytest.raises(TargetModeMismatchError):
        planner.inverse_kinematics(target, robot_cell_state)
    with pytest.raises(TargetModeMismatchError):
        planner.forward_kinematics(robot_cell_state, TargetMode.TOOL)
    # ROBOT target mode should be possible
    target = FrameTarget(target_frame, TargetMode.ROBOT)
    planner.inverse_kinematics(target, robot_cell_state, options=options)
    planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)


def test_iter_ik_frame_target(planner_with_test_cell):
    planner, robot_cell, robot_cell_state = planner_with_test_cell

    # Basic test to produce results

    options = {
        "max_results": 1,
        "check_collision": True,
        "verbose": False,
    }

    # This target is reachable and collision free
    target = FrameTarget(Frame([0.5, 0.5, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), TargetMode.ROBOT)

    # Even this is an internal function:
    # Test that if someone forgot to provide a group or an invalid one, it will raise an error
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


def test_iter_ik_returns_different_results(planner_with_test_cell):
    planner, robot_cell, robot_cell_state = planner_with_test_cell

    # Testing conditions
    group = robot_cell.robot.main_group_name
    options = {
        "check_collision": True,
        "solution_uniqueness_threshold_prismatic": 1e-3,
        "solution_uniqueness_threshold_revolute": 1e-3,
    }

    # Construct the IK target
    target = FrameTarget(
        target_frame=Frame([0.5, 0.5, 0.5], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        target_mode=TargetMode.ROBOT,
        tolerance_position=1e-3,
        tolerance_orientation=1e-3,
    )
    # Plan IK to get configuration
    generator = planner.iter_inverse_kinematics(target, robot_cell_state, group=group, options=options)
    result1 = next(generator)
    result2 = next(generator)
    assert isinstance(result1, Configuration)
    assert isinstance(result2, Configuration)
    assert not compare_configuration(result1, result2, 1e-3, 1e-3)
    result3 = next(generator)
    assert not compare_configuration(result1, result3, 1e-3, 1e-3)
    assert not compare_configuration(result2, result3, 1e-3, 1e-3)


def test_iter_ik_point_axis_target(planner_with_test_cell):
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

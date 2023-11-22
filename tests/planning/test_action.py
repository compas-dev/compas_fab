import pytest


from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Transformation
from compas.geometry import Translation

from compas.robots import Configuration

from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint

from compas_fab.planning import Action
from compas_fab.planning import RoboticAction
from compas_fab.planning import CartesianMotion
from compas_fab.planning import FreeMotion
from compas_fab.planning import OpenGripper
from compas_fab.planning import CloseGripper
from compas_fab.planning import ManuallyMoveWorkpiece

from compas_fab.planning import SceneState
from compas_fab.planning import WorkpieceState
from compas_fab.planning import ToolState
from compas_fab.planning import RobotState


@pytest.fixture
def robot_state():
    return RobotState(
        frame=Frame.worldXY(),
        configuration=Configuration.from_revolute_values(
            [10, 20, 30, 40, 50, 60], ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        ),
    )


@pytest.fixture
def start_scene_state(robot_state):
    # type: (RobotState) -> SceneState
    scene_state = SceneState(
        workpiece_ids=["w1", "w2"],
        tool_ids=["t1", "t2"],
    )
    scene_state.workpiece_states["w1"] = WorkpieceState(
        "w1", Frame(Point(10.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0))
    )
    scene_state.workpiece_states["w2"] = WorkpieceState(
        "w2", Frame(Point(20.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0))
    )
    scene_state.tool_states["t1"] = ToolState("t1", Frame.worldYZ())
    scene_state.tool_states["t2"] = ToolState("t2", Frame.worldZX())
    # T1 is attached to the robot and the attachment point has an offset of 100mm in z-direction
    scene_state.tool_states["t1"].attached_to_robot = True
    scene_state.tool_states["t1"].attached_to_robot_grasp = Translation.from_vector([0, 0, 100])
    # W1 is attached to T1 and the attachment direction is rotated by -90 degrees around the Y-axis
    scene_state.workpiece_states["w1"].attached_to_robot = True
    scene_state.workpiece_states["w1"].attached_to_robot_grasp = Transformation.from_frame(Frame.worldYZ())
    scene_state.robot_state = robot_state
    return scene_state


@pytest.fixture
def jtp():
    return JointTrajectoryPoint(
        [1.571, 0, 0, 0.262, 0, 0],
        [0] * 6,
        [3.0] * 6,
        time_from_start=Duration(2, 1293),
        joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
    )


@pytest.fixture
def trj():
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    p1 = JointTrajectoryPoint([1.571, 0, 0, 0.262, 0, 0], [0] * 6, [3.0] * 6, time_from_start=Duration(2, 1293))
    p2 = JointTrajectoryPoint([0.571, 0, 0, 0.262, 0, 0], [0] * 6, [3.0] * 6, time_from_start=Duration(6, 0))
    start_configuration = Configuration.from_prismatic_and_revolute_values(
        p1.prismatic_values, p1.revolute_values, joint_names
    )

    return JointTrajectory(
        trajectory_points=[p1, p2], joint_names=joint_names, start_configuration=start_configuration, fraction=1.0
    )


def test_action_members():
    # Test child class of Action
    assert isinstance(RoboticAction(), Action)
    assert isinstance(LinearMotion(), Action)
    assert isinstance(FreeMotion(), Action)
    assert isinstance(OpenGripper(), Action)
    assert isinstance(CloseGripper(), Action)
    assert isinstance(ManuallyMoveWorkpiece(), Action)
    # Test child class of RoboticAction
    assert isinstance(LinearMotion(), RoboticAction)
    assert isinstance(FreeMotion(), RoboticAction)


def test_robot_action_serialization(trj, jtp):
    action = RoboticAction()
    action.target_robot_flange_frame = Frame.worldXY()
    action.allowed_collision_pairs = [("a1", "b1"), ("c1", "d1")]
    action.fixed_target_configuration = Configuration.from_data(jtp.data)
    action.fixed_trajectory = trj
    data = action.to_data()
    new_action = RoboticAction.from_data(data)
    assert new_action.to_data() == data
    assert new_action.target_robot_flange_frame == Frame.worldXY()
    assert ("a1", "b1") in new_action.allowed_collision_pairs
    assert ("c1", "d1") in new_action.allowed_collision_pairs
    assert new_action.fixed_target_configuration["joint_1"] == 1.571
    trajectory_point = new_action.fixed_trajectory.points[1]
    full_config = Configuration.from_prismatic_and_revolute_values(
        trajectory_point.prismatic_values, trajectory_point.revolute_values, new_action.fixed_trajectory.joint_names
    )
    assert full_config["joint_1"] == 0.571
    assert new_action.fixed_trajectory.time_from_start == Duration(6, 0).seconds


def test_robotic_motion_effect(start_scene_state):
    motion = RoboticAction()
    # Target frame is on the YZ plane of the world
    target_flange_frame = Frame(Point(100.0, 200.0, 300.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0))
    motion.target_robot_flange_frame = target_flange_frame
    scene_state = start_scene_state.copy()
    motion.apply_effects(scene_state)

    # Robot Frame
    assert scene_state.robot_state.frame == target_flange_frame
    # Attached Tool
    assert scene_state.get_attached_tool_id() == "t1"
    grasp = Translation.from_vector([0, 0, 100])
    assert scene_state.get_tool_state("t1").attached_to_robot_grasp == grasp
    tool_frame = Frame(Point(200.000, 200.000, 300.000), Vector(0.000, 1.000, 0.000), Vector(0.000, 0.000, 1.000))
    assert scene_state.get_tool_state("t1").frame == tool_frame
    # Stationary Tool
    assert scene_state.get_tool_state("t2").frame == Frame.worldZX()
    # Attached Workpiece
    assert scene_state.get_attached_workpiece_id() == "w1"
    assert scene_state.get_workpiece_state("w1").attached_to_robot_grasp == Transformation.from_frame(Frame.worldYZ())
    # Note that the workpiece frame is not stacked on top of the tool frame transformation
    workpiece_frame = Frame(
        Point(100.000, 200.000, 300.000), Vector(0.000, 0.000, 1.000), Vector(1.000, -0.000, -0.000)
    )
    assert scene_state.get_workpiece_state("w1").frame == workpiece_frame


def test_open_gripper_effect(start_scene_state):
    action = OpenGripper()
    action.apply_effects(start_scene_state)
    assert start_scene_state.get_attached_workpiece_id() is None
    assert start_scene_state.workpiece_states["w1"].attached_to_robot is False
    assert start_scene_state.workpiece_states["w2"].attached_to_robot is False


def test_close_gripper_effect(start_scene_state):
    # Set that no objects are attached to the robot
    start_scene_state.workpiece_states["w1"].attached_to_robot = False
    start_scene_state.workpiece_states["w2"].attached_to_robot = False
    robot_flange_frame = Frame(Point(10.0, 20.0, 30.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0))
    start_scene_state.robot_state.frame = robot_flange_frame
    # Attach workpiece w1 to the robot
    grasp = Translation.from_vector([0, 0, 10])
    action = CloseGripper("t1", "w1", grasp)
    action.apply_effects(start_scene_state)
    # Check that the workpiece w1 is attached to the robot
    assert start_scene_state.get_attached_workpiece_id() == "w1"
    assert start_scene_state.workpiece_states["w1"].attached_to_robot is True
    assert start_scene_state.workpiece_states["w1"].attached_to_robot_grasp == grasp
    workpiece_frame = Frame(Point(20.0, 20.0, 30.0), Vector(0.0, 1.0, 0.0), Vector(0.0, 0.0, 1.0))
    assert start_scene_state.workpiece_states["w1"].frame == workpiece_frame
    # Workpiece w2 remains unattached
    assert start_scene_state.workpiece_states["w2"].attached_to_robot is False


def test_open_gripper_preconditions(start_scene_state):
    action = OpenGripper("t1")
    assert action.check_preconditions(start_scene_state)[0] is True
    # Detach the tool from the robot and check that the preconditions are not met
    start_scene_state.tool_states["t1"].attached_to_robot = False
    assert action.check_preconditions(start_scene_state)[0] is False
    # Attach a wrong tool to the robot and check that the preconditions are not met
    start_scene_state.tool_states["t2"].attached_to_robot = True
    assert action.check_preconditions(start_scene_state)[0] is False


def test_close_gripper_preconditions(start_scene_state):
    action = CloseGripper("t1", "w1")
    # Default state object has one object attached to the robot
    assert start_scene_state.get_attached_tool_id() == "t1"
    assert start_scene_state.get_attached_workpiece_id() is not None
    assert action.check_preconditions(start_scene_state)[0] is False

    # Set that no objects are attached to the robot
    start_scene_state.workpiece_states["w1"].attached_to_robot = False
    start_scene_state.workpiece_states["w2"].attached_to_robot = False
    # Attach workpiece w1 to the robot
    assert action.check_preconditions(start_scene_state)[0] is True

    # Detach the tool from the robot and check that the preconditions are not met
    start_scene_state.tool_states["t1"].attached_to_robot = False
    assert action.check_preconditions(start_scene_state)[0] is False

    # Attach workpiece w2 to the robot and check that the preconditions are not met
    start_scene_state.tool_states["t1"].attached_to_robot = True
    start_scene_state.workpiece_states["w2"].attached_to_robot = True
    assert action.check_preconditions(start_scene_state)[0] is False

    # Attach a wrong tool to the robot and check that the preconditions are not met
    action = CloseGripper("t1", "w1")
    start_scene_state.tool_states["t1"].attached_to_robot = False
    start_scene_state.tool_states["t2"].attached_to_robot = True
    start_scene_state.workpiece_states["w1"].attached_to_robot = False
    start_scene_state.workpiece_states["w2"].attached_to_robot = False
    assert action.check_preconditions(start_scene_state)[0] is False

    action = CloseGripper("t2", "w2")
    assert action.check_preconditions(start_scene_state)[0] is True

    # Check non-existing tool
    action = CloseGripper("t3", "w2")
    assert action.check_preconditions(start_scene_state)[0] is False

    # Check non-existing workpiece
    action = CloseGripper("t2", "w3")
    assert action.check_preconditions(start_scene_state)[0] is False


def test_load_workpiece_preconditions(start_scene_state):
    action = ManuallyMoveWorkpiece("w1", None)
    # Default state object has one object attached to the robot
    assert start_scene_state.get_attached_tool_id() == "t1"
    assert start_scene_state.get_attached_workpiece_id() == "w1"
    assert action.check_preconditions(start_scene_state)[0] is False

    # Detach the workpiece from the robot
    start_scene_state.workpiece_states["w1"].attached_to_robot = False
    # ManuallyMoveWorkpiece action requires that the workpiece is not attached to the robot
    assert action.check_preconditions(start_scene_state)[0] is True

    # Check non-existing workpiece
    action = ManuallyMoveWorkpiece("w3", None)
    assert action.check_preconditions(start_scene_state)[0] is False


def test_load_workpiece_effect(start_scene_state):
    workpiece_frame = Frame(Point(100.0, 200.0, 300.0), Vector(1.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0))
    assert start_scene_state.get_attached_workpiece_id() == "w1"
    action = ManuallyMoveWorkpiece("w2", workpiece_frame)

    action.apply_effects(start_scene_state)
    # Workpiece w2 should remain unattached but moved
    assert start_scene_state.workpiece_states["w2"].attached_to_robot is False
    assert start_scene_state.workpiece_states["w2"].frame == workpiece_frame
    # Workpiece w1 should remain attached
    assert start_scene_state.get_attached_workpiece_id() == "w1"

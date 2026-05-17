import os
import threading

import pytest
from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import axis_angle_from_quaternion
from roslibpy import Message
from roslibpy import Topic

from compas_fab.backends import InverseKinematicsError
from compas_fab.backends import MotionPlanningError
from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import FrameTarget
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyLibrary
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode
from compas_fab.robots import ToolLibrary


RUN_ENV_VAR = "COMPAS_FAB_RUN_ROS_INTEGRATION_TESTS"
ROS_HOST_ENV_VAR = "COMPAS_FAB_ROS_HOST"
ROS_PORT_ENV_VAR = "COMPAS_FAB_ROS_PORT"


@pytest.fixture(scope="module")
def ros_client():
    if os.environ.get(RUN_ENV_VAR) != "1":
        pytest.skip("ROS integration tests are opt-in. Start tests/integration_setup/docker-compose.yml and set {}=1 to run the ROS docs examples as tests.".format(RUN_ENV_VAR))

    host = os.environ.get(ROS_HOST_ENV_VAR, "localhost")
    port = int(os.environ.get(ROS_PORT_ENV_VAR, "9090"))
    client = RosClient(host=host, port=port)

    try:
        client.run(timeout=5)
    except Exception as e:
        pytest.skip("Could not connect to rosbridge at {}:{}: {}".format(host, port, e))

    yield client
    client.close()


@pytest.fixture()
def ur5_robot_cell(ros_client):
    robot_cell = ros_client.load_robot_cell(load_geometry=False)
    if robot_cell.robot_model.name != "ur5_robot":
        pytest.skip("The ROS docs examples target the Noetic UR5 integration stack; connected to {!r} instead.".format(robot_cell.robot_model.name))
    return robot_cell


@pytest.fixture()
def planner(ros_client):
    return MoveItPlanner(ros_client)


def assert_frame_close(frame, expected_point, tolerance=1e-2):
    assert frame.point.distance_to_point(expected_point) <= tolerance


def test_01_ros_connection_example(ros_client):
    assert ros_client.is_connected
    assert ros_client.ros_distro.value


def test_02_load_robot_cell_example(ur5_robot_cell, planner):
    assert planner.client.robot_cell is ur5_robot_cell
    assert ur5_robot_cell.root_name == "base_link"
    assert ur5_robot_cell.main_group_name == "manipulator"
    assert ur5_robot_cell.get_end_effector_link_name() == "tool0"
    assert "shoulder_pan_joint" in [joint.name for joint in ur5_robot_cell.robot_model.iter_joints()]
    assert "tool0" in ur5_robot_cell.get_link_names()


def test_02_set_robot_cell_example(ur5_robot_cell, planner):
    gripper = ToolLibrary.static_gripper_small()
    ur5_robot_cell.tool_models["my_gripper"] = gripper

    beam_mesh = Box(1, 0.1, 0.1).to_mesh(triangulated=True)
    ur5_robot_cell.rigid_body_models["my_beam"] = RigidBody.from_mesh(beam_mesh)
    ur5_robot_cell.rigid_body_models["my_floor"] = RigidBodyLibrary.floor()

    planner.set_robot_cell(ur5_robot_cell)


def test_02_set_robot_cell_state_with_attached_objects_example(ur5_robot_cell, planner):
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_gripper_one_beam()
    planner.set_robot_cell(robot_cell)

    robot_cell_state.rigid_body_states["beam"].frame = Frame([1.2, 0, 0], [0, 0, 1], [1, 0, 0])
    robot_cell_state.tool_states["gripper"].attached_to_group = robot_cell.main_group_name
    robot_cell_state.tool_states["gripper"].attachment_frame = Frame([0, 0, 0], [0, 0, 1], [1, 0, 0])
    planner.set_robot_cell_state(robot_cell_state)

    robot_cell_state.rigid_body_states["beam"].attached_to_tool = "gripper"
    robot_cell_state.rigid_body_states["beam"].attachment_frame = Frame([0, 0, 0], [1, 0, 0], [0, 1, 0])
    planner.set_robot_cell_state(robot_cell_state)

    robot_cell_state.robot_configuration.joint_values[1] = -1.0
    robot_cell_state.robot_configuration.joint_values[2] = 0.5
    planner.set_robot_cell_state(robot_cell_state)


def test_02_set_robot_cell_state_with_kinematic_tool_example(ur5_robot_cell, planner):
    gripper = ToolLibrary.kinematic_gripper()
    ur5_robot_cell.tool_models[gripper.name] = gripper

    robot_cell_state = ur5_robot_cell.default_cell_state()
    robot_cell_state.set_tool_attached_to_group(
        gripper.name,
        ur5_robot_cell.main_group_name,
        attachment_frame=Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        touch_links=["wrist_3_link"],
    )
    robot_cell_state.robot_configuration.joint_values[1] = -1.0
    robot_cell_state.robot_configuration.joint_values[2] = 0.5
    planner.set_robot_cell(ur5_robot_cell, robot_cell_state)

    opened_gripper_configuration = gripper.zero_configuration()
    opened_gripper_configuration.joint_values = [0.025, 0.025]
    robot_cell_state.tool_states[gripper.name].configuration = opened_gripper_configuration
    planner.set_robot_cell_state(robot_cell_state)


def test_03_forward_kinematics_examples(ur5_robot_cell, planner):
    robot_cell_state = ur5_robot_cell.default_cell_state()
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    frame = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)
    assert_frame_close(frame, Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]).point)

    for link_name in ur5_robot_cell.get_link_names():
        link_frame = planner.forward_kinematics_to_link(robot_cell_state, link_name)
        assert isinstance(link_frame, Frame)


def test_03_forward_kinematics_target_mode_example(ros_client, ur5_robot_cell, planner):
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_gripper_one_beam(ros_client)
    ros_client.load_robot_cell(False)
    assert ros_client.robot_cell.root_name == robot_cell.root_name

    planner.set_robot_cell(robot_cell)
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]
    robot_cell_state.rigid_body_states["beam"].attachment_frame = Frame([0.0, 0.0, -0.1], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])

    robot_frame = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)
    tool_frame = planner.forward_kinematics(robot_cell_state, TargetMode.TOOL)
    workpiece_frame = planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)

    assert_frame_close(robot_frame, Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]).point)
    assert_frame_close(tool_frame, Frame([0.35, 0.1, 0.5], [1, 0, 0], [0, 1, 0]).point)
    assert_frame_close(workpiece_frame, Frame([0.35, 0.1, 0.6], [1, 0, 0], [0, 1, 0]).point)


def test_04_inverse_kinematics_examples(ur5_robot_cell, planner):
    start_state = ur5_robot_cell.default_cell_state()
    target = FrameTarget(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]), TargetMode.ROBOT)

    configuration = planner.inverse_kinematics(target, start_state, options={"max_results": 1})
    assert len(configuration.joint_values) == 6

    target.target_frame.point.z += 0.1
    start_state.robot_configuration.merge(configuration)
    configuration = planner.inverse_kinematics(target, start_state, options={"max_results": 1})
    assert len(configuration.joint_values) == 6


def test_04_inverse_kinematics_full_configuration_example(ur5_robot_cell, planner):
    start_state = ur5_robot_cell.default_cell_state()
    target = FrameTarget(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]), TargetMode.ROBOT)

    configuration = planner.inverse_kinematics(target, start_state, options={"max_results": 1, "return_full_configuration": True})

    assert len(configuration.joint_values) == len(ur5_robot_cell.zero_full_configuration().joint_values)


def test_04_inverse_kinematics_unreachable_example(ur5_robot_cell, planner):
    start_state = ur5_robot_cell.default_cell_state()
    target = FrameTarget(Frame([2.0, 1.0, 1.0], [1, 0, 0], [0, 1, 0]), TargetMode.ROBOT)

    with pytest.raises(InverseKinematicsError):
        planner.inverse_kinematics(target, start_state, options={"max_results": 1, "timeout": 1})


def test_04_inverse_kinematics_allow_collision_example(ur5_robot_cell, planner):
    box_mesh = Box(0.1, 0.1, 0.6).to_mesh(triangulated=True)
    ur5_robot_cell.rigid_body_models["box"] = RigidBody.from_mesh(box_mesh)

    start_state = ur5_robot_cell.default_cell_state()
    start_state.rigid_body_states["box"].frame.point = [0.3, 0.1, 0.3]
    planner.set_robot_cell(ur5_robot_cell, start_state)

    target = FrameTarget(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]), TargetMode.ROBOT)

    with pytest.raises(InverseKinematicsError):
        planner.inverse_kinematics(target, start_state, options={"max_results": 1})

    configuration = planner.inverse_kinematics(target, start_state, options={"max_results": 1, "allow_collision": True})
    assert len(configuration.joint_values) == 6


def test_04_iter_inverse_kinematics_examples(ur5_robot_cell, planner):
    start_state = ur5_robot_cell.default_cell_state()
    target = FrameTarget(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]), TargetMode.ROBOT)

    configurations = list(planner.iter_inverse_kinematics(target, start_state, options={"max_results": 2}))

    assert configurations
    assert all(len(configuration.joint_values) == 6 for configuration in configurations)


def test_05_plan_motion_frame_target_example(ur5_robot_cell, planner):
    start_state = ur5_robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]

    target = FrameTarget(Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1]), TargetMode.ROBOT)
    trajectory = planner.plan_motion(target, start_state, options={"allowed_planning_time": 2.0})

    assert len(trajectory.points) >= 2
    assert trajectory.time_from_start > 0


def test_05_plan_motion_configuration_target_example(ur5_robot_cell, planner):
    start_state = ur5_robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]

    target_configuration = ur5_robot_cell.zero_full_configuration()
    target_configuration.joint_values = [-2.132, 4.645, -1.582, -2.903, 5.758, -0.050]
    target = ConfigurationTarget(target_configuration, tolerance_above=0.001, tolerance_below=0.001)
    trajectory = planner.plan_motion(target, start_state, options={"allowed_planning_time": 2.0})

    assert len(trajectory.points) >= 2
    assert trajectory.points[-1].joint_values


def test_05_plan_motion_tolerance_example(ur5_robot_cell, planner):
    start_state = ur5_robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]

    target = FrameTarget(Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1]), TargetMode.ROBOT)
    target.tolerance_position = 0.0001
    target.tolerance_orientation = 0.001
    trajectory = planner.plan_motion(target, start_state, options={"allowed_planning_time": 2.0})

    end_state = start_state.copy()
    end_state.robot_configuration.joint_values = trajectory.points[-1].joint_values
    robot_frame = planner.forward_kinematics(end_state, TargetMode.ROBOT)
    delta_frame = robot_frame.to_local_coordinates(target.target_frame)
    _, orientation_error = axis_angle_from_quaternion(delta_frame.quaternion)

    assert target.target_frame.point.distance_to_point(robot_frame.point) <= 0.001
    assert orientation_error <= 0.01


def test_05_plan_motion_with_obstacle_example(ur5_robot_cell, planner):
    box_mesh = Box(0.1, 0.1, 1.2).to_mesh(triangulated=True)
    ur5_robot_cell.rigid_body_models["box"] = RigidBody.from_mesh(box_mesh)
    planner.set_robot_cell(ur5_robot_cell)

    start_state = ur5_robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]
    start_state.rigid_body_states["box"].frame.point = [0.3, 0.0, 0.0]

    target = FrameTarget(Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1]), TargetMode.ROBOT)
    trajectory = planner.plan_motion(target, start_state, options={"allowed_planning_time": 2.0})

    assert len(trajectory.points) >= 2


def test_06_cartesian_motion_examples(ur5_robot_cell, planner):
    robot_cell_state = ur5_robot_cell.default_cell_state()
    robot_cell_state.robot_configuration.joint_values = (-0.042, 0.033, -2.174, 5.282, -1.528, 0.000)

    waypoints = FrameWaypoints(
        [
            Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]),
            Frame([0.5, 0.1, 0.6], [1, 0, 0], [0, 1, 0]),
        ],
        TargetMode.ROBOT,
    )
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state)

    assert len(trajectory.points) >= 2
    assert trajectory.fraction == pytest.approx(1.0)
    assert trajectory.time_from_start > 0


def test_06_cartesian_motion_step_size_example(ur5_robot_cell, planner):
    robot_cell_state = ur5_robot_cell.default_cell_state()
    robot_cell_state.robot_configuration.joint_values = (-0.042, 0.033, -2.174, 5.282, -1.528, 0.000)
    waypoints = FrameWaypoints([Frame([0.3, 0.3, 0.5], [0, -1, 0], [0, 0, -1])], TargetMode.ROBOT)

    coarse_trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options={"max_step": 0.01})
    fine_trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options={"max_step": 0.001})

    assert len(fine_trajectory.points) > len(coarse_trajectory.points)


def test_06_cartesian_motion_partial_example(ur5_robot_cell, planner):
    box_mesh = Box(0.1, 0.1, 1.2).to_mesh(triangulated=True)
    ur5_robot_cell.rigid_body_models["box"] = RigidBody.from_mesh(box_mesh)
    planner.set_robot_cell(ur5_robot_cell)

    start_state = ur5_robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]
    start_state.rigid_body_states["box"].frame.point = [0.3, 0.0, 0.0]
    waypoints = FrameWaypoints([Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])], TargetMode.ROBOT)

    with pytest.raises(MotionPlanningError) as error:
        planner.plan_cartesian_motion(waypoints, start_state)

    assert error.value.partial_trajectory is not None
    assert 0 < error.value.partial_trajectory.fraction < 1


def test_06_cartesian_motion_target_mode_example(ros_client, ur5_robot_cell, planner):
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_gripper_one_beam(ros_client)
    ros_client.load_robot_cell(False)
    assert ros_client.robot_cell.root_name == robot_cell.root_name

    planner.set_robot_cell(robot_cell)
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    start_frame = planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)
    waypoints = FrameWaypoints(
        [
            Frame([0.35, 0.3, 0.5], [1, 0, 0], [0, -1, 0]),
            Frame([0.35, 0.3, 0.7], [1, 0, 0], [0, -1, 0]),
            Frame([0.35, 0.1, 0.7], [1, 0, 0], [0, -1, 0]),
            start_frame,
        ],
        TargetMode.WORKPIECE,
    )
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state)

    assert len(trajectory.points) >= 2
    assert trajectory.fraction == pytest.approx(1.0)


def test_10_ros_hello_world_pubsub_example(ros_client):
    received = []
    received_event = threading.Event()

    def receive_message(message):
        received.append(message["data"])
        received_event.set()

    listener = Topic(ros_client, "/chatter", "std_msgs/String")
    talker = Topic(ros_client, "/chatter", "std_msgs/String")
    listener.subscribe(receive_message)

    try:
        talker.publish(Message({"data": "Hello World!"}))
        assert received_event.wait(5)
    finally:
        listener.unsubscribe()
        talker.unadvertise()

    assert received == ["Hello World!"]

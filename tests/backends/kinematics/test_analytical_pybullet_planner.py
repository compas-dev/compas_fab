import pytest

import compas
from compas.geometry import Frame
from compas_robots import Configuration

import compas_fab
from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends import UR5Kinematics
from compas_fab.robots import Tool
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotLibrary
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode
from compas_fab.robots import FrameTarget


if not compas.IPY:
    from compas_fab.backends import AnalyticalPyBulletPlanner
    from compas_fab.backends import AnalyticalPyBulletClient

urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
srdf_filename = compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf")


@pytest.fixture
def analytical_pybullet_client():
    with AnalyticalPyBulletClient(connection_type="direct") as client:
        yield client


def test_planner(analytical_pybullet_client):
    if compas.IPY:
        return

    client = analytical_pybullet_client
    planner = AnalyticalPyBulletPlanner(client, UR5Kinematics())
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()

    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)

    assert planner.client is not None
    assert client.robot_cell is not None
    assert client.robot is not None
    assert planner.kinematics_solver is not None


def forward_inverse_agreement(planner, start_state, options=None):
    # type: (AnalyticalPyBulletPlanner, RobotCellState, Optional[Dict]) -> None
    """Helper function to test forward and inverse kinematics agreement"""

    # First perform forward kinematics to find the frame
    starting_configuration = start_state.robot_configuration
    frame = planner.forward_kinematics(start_state, TargetMode.ROBOT, group=None)

    # Second, perform Inverse Kinematics to see if any of the solutions
    # are the same as the original joint configuration
    # Because there are 8 possible solutions, we can break early if we find a close solution

    start_state = RobotCellState.from_robot_cell(planner.client.robot_cell)
    ik_configurations = list(
        planner.iter_inverse_kinematics(FrameTarget(frame, TargetMode.ROBOT), start_state, group=None, options=options)
    )
    for configuration in ik_configurations:
        if configuration.close_to(starting_configuration):
            print("Found a close solution: {}".format(configuration))
            return True
    assert False


def test_forward_inverse_agreement_ur5(analytical_pybullet_client):
    """Test that the forward and inverse kinematics are in agreement"""
    if compas.IPY:
        return
    # It is a known problem that the kinematics in AnalyticalKinematicsPlanner does not always
    # return the same result as the kinematics in the RobotModel.

    client = analytical_pybullet_client
    planner = AnalyticalPyBulletPlanner(client, UR5Kinematics())
    robot = RobotLibrary.ur5()
    robot_cell = RobotCell(robot)
    robot_cell_state = RobotCellState.from_robot_cell(robot_cell)
    planner.set_robot_cell(robot_cell, robot_cell_state)

    # TODO: This problem will be solved after we improved the AnalyticalKinematics class
    # with base and tip offsets, as well as joint value offsets.
    # This will allow us to compare the results of the AnalyticalKinematics with the RobotModel

    # This test starts with FK and then uses IK to find the original joint configuration
    start_state = RobotCellState.from_robot_cell(robot_cell)

    options = {"check_collision": False, "keep_order": False}

    # All these joint values should be positive and smaller than 2*pi
    start_state.robot_configuration.joint_values = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state, options)

    start_state.robot_configuration.joint_values = [0.6, 0.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state, options)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 1.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state, options)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 1.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state, options)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 2.6, 2.3]
    forward_inverse_agreement(planner, start_state, options)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 4.6, 2.3]
    forward_inverse_agreement(planner, start_state, options)

    start_state.robot_configuration.joint_values = [0.6, 2.5, 3.4, 4.3, 4.6, 4.3]
    forward_inverse_agreement(planner, start_state, options)


def test_iter_inverse_kinematics(analytical_pybullet_client):

    client = analytical_pybullet_client  # type: AnalyticalPyBulletClient
    planner = AnalyticalPyBulletPlanner(client, UR5Kinematics())
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()

    planner.set_robot_cell(robot_cell, robot_cell_state)

    # This target has eight solutions (without CC)
    target = FrameTarget(
        target_frame=Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269)),
        target_mode=TargetMode.ROBOT,
    )

    # The `iter_inverse_kinematics` method will return an iterator that yields all possible solutions
    options = {"keep_order": True}
    solutions = list(planner.iter_inverse_kinematics(target, robot_cell_state, group=None, options=options))
    assert len(solutions) == 8

    options = {"check_collision": False, "keep_order": True}
    solutions = list(planner.iter_inverse_kinematics(target, robot_cell_state, group=None, options=options))
    assert len(solutions) == 8

    options = {"check_collision": True, "keep_order": False}
    solutions = list(planner.iter_inverse_kinematics(target, robot_cell_state, group=None, options=options))
    assert len(solutions) < 8

    print(solutions)
    ground_truth = Configuration.from_revolute_values((0.022, 4.827, 1.508, 1.126, 1.876, 3.163))
    assert any(configuration.close_to(ground_truth) for configuration in solutions)


# def test_kinematics_client():
#     if compas.IPY:
#         return
#     frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
#     with AnalyticalPyBulletClient(connection_type="direct") as client:
#         robot = client.load_robot(urdf_filename)
#         client.load_semantics(robot, srdf_filename)
#         solutions = list(
#             client.iter_inverse_kinematics(
#                 frame_WCF, options={"solver": "ur5", "check_collision": True, "keep_order": False}
#             )
#         )
#         assert len(solutions) == 5


# def test_kinematics_client_with_attached_tool_but_disabled_usage_of_tcf():
#     if compas.IPY:
#         return
#     frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
#     with AnalyticalPyBulletClient(connection_type="direct") as client:
#         robot = client.load_robot(urdf_filename)
#         client.load_semantics(robot, srdf_filename)

#         tool_frame = Frame([0.0, 0, 0], [0, 1, 0], [0, 0, 1])
#         robot.attach_tool(Tool(visual=None, frame_in_tool0_frame=tool_frame))

#         solutions = list(
#             robot.iter_inverse_kinematics(
#                 frame_WCF,
#                 use_attached_tool_frame=False,
#                 options={"solver": "ur5", "check_collision": True, "keep_order": False},
#             )
#         )
#         assert len(solutions) == 5


# def test_kinematics_client_with_attached_tool():
#     if compas.IPY:
#         return
#     frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
#     with AnalyticalPyBulletClient(connection_type="direct") as client:
#         robot = client.load_robot(urdf_filename)
#         client.load_semantics(robot, srdf_filename)

#         tool_frame = Frame([0, 0, 0.6], [1, 0, 0], [0, 1, 0])
#         robot.attach_tool(Tool(visual=None, frame_in_tool0_frame=tool_frame))

#         solutions = list(
#             robot.iter_inverse_kinematics(
#                 frame_WCF, options={"solver": "ur5", "check_collision": True, "keep_order": False}
#             )
#         )
#         assert len(solutions) == 6


# @pytest.fixture
# def frames_WCF():
#     """A list of frames in the world coordinate frame for planning tests"""
#     return [
#         Frame((0.407, 0.073, 0.320), (0.922, 0.000, 0.388), (0.113, 0.956, -0.269)),
#         Frame((0.404, 0.057, 0.324), (0.919, 0.000, 0.394), (0.090, 0.974, -0.210)),
#         Frame((0.390, 0.064, 0.315), (0.891, 0.000, 0.454), (0.116, 0.967, -0.228)),
#         Frame((0.388, 0.079, 0.309), (0.881, 0.000, 0.473), (0.149, 0.949, -0.278)),
#         Frame((0.376, 0.087, 0.299), (0.850, 0.000, 0.528), (0.184, 0.937, -0.296)),
#     ]


# @pytest.fixture
# def frame_waypoints(frames_WCF):
#     """A FrameWaypoints Object for planning tests"""
#     return FrameWaypoints(frames_WCF)


# def test_kinematics_cartesian(frame_waypoints):
#     if compas.IPY:
#         return

#     with AnalyticalPyBulletClient(connection_type="direct") as client:
#         robot = client.load_robot(urdf_filename)
#         client.load_semantics(robot, srdf_filename)

#         options = {"solver": "ur5", "check_collision": True}

#         trajectory = robot.plan_cartesian_motion(frame_waypoints, options=options)

#         # Assert that the trajectory is complete
#         assert trajectory.fraction == 1.0
#         # Assert that the trajectory has the correct number of points
#         assert len(trajectory.points) == len(frame_waypoints.target_frames)


# def test_kinematics_cartesian_with_tool_coordinate_frame(frame_waypoints):
#     if compas.IPY:
#         return
#     tool_coordinate_frame = Frame([0.01, 0.02, -0.03], [1, 0, 0], [0, 1, 0])

#     with AnalyticalPyBulletClient(connection_type="direct") as client:
#         robot = client.load_robot(urdf_filename)
#         client.load_semantics(robot, srdf_filename)

#         options = {"solver": "ur5", "check_collision": True}

#         trajectory = robot.plan_cartesian_motion(frame_waypoints, options=options)

#         # Assert that the trajectory is complete
#         assert trajectory.fraction == 1.0
#         # Assert that the trajectory has the correct number of points
#         # NOTE: At the moment the AnalyticalPyBulletClient does not perform any interpolation between frames
#         # NOTE: if future implementation fixes this, the following test will not be valid anymore
#         assert len(trajectory.points) == len(frame_waypoints.target_frames)

import pytest

import compas
from compas.geometry import Frame
from compas_robots import Configuration

import compas_fab
from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.robots import Tool
from compas_fab.robots import RobotLibrary
from compas_fab.robots import FrameWaypoints

if not compas.IPY:
    from compas_fab.backends import AnalyticalPyBulletClient

urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
srdf_filename = compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf")


def test_inverse_kinematics():
    ik = AnalyticalInverseKinematics()
    robot = RobotLibrary.ur5()
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    # set the solver later
    solutions = list(
        ik.inverse_kinematics(robot, frame_WCF, start_configuration=None, group=None, options={"solver": "ur5"})
    )
    assert len(solutions) == 8
    joint_positions, _ = solutions[0]
    correct = Configuration.from_revolute_values((0.022, 4.827, 1.508, 1.126, 1.876, 3.163))
    assert correct.close_to(Configuration.from_revolute_values(joint_positions))


def test_inverse_kinematics2():
    ik = AnalyticalInverseKinematics(solver="ur5")  # set the solver in the beginning
    robot = RobotLibrary.ur5()
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    # set the solver later
    solutions = list(ik.inverse_kinematics(robot, frame_WCF, start_configuration=None, group=None))
    assert len(solutions) == 8
    joint_positions, _ = solutions[0]
    correct = Configuration.from_revolute_values((0.022, 4.827, 1.508, 1.126, 1.876, 3.163))
    assert correct.close_to(Configuration.from_revolute_values(joint_positions))


def test_kinematics_client():
    if compas.IPY:
        return
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    with AnalyticalPyBulletClient(connection_type="direct") as client:
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)
        solutions = list(
            client.iter_inverse_kinematics(
                frame_WCF, options={"solver": "ur5", "check_collision": True, "keep_order": False}
            )
        )
        assert len(solutions) == 5


def test_kinematics_client_with_attached_tool_but_disabled_usage_of_tcf():
    if compas.IPY:
        return
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    with AnalyticalPyBulletClient(connection_type="direct") as client:
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)

        tool_frame = Frame([0.0, 0, 0], [0, 1, 0], [0, 0, 1])
        robot.attach_tool(Tool(visual=None, frame_in_tool0_frame=tool_frame))

        solutions = list(
            robot.iter_inverse_kinematics(
                frame_WCF,
                use_attached_tool_frame=False,
                options={"solver": "ur5", "check_collision": True, "keep_order": False},
            )
        )
        assert len(solutions) == 5


def test_kinematics_client_with_attached_tool():
    if compas.IPY:
        return
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    with AnalyticalPyBulletClient(connection_type="direct") as client:
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)

        tool_frame = Frame([0, 0, 0.6], [1, 0, 0], [0, 1, 0])
        robot.attach_tool(Tool(visual=None, frame_in_tool0_frame=tool_frame))

        solutions = list(
            robot.iter_inverse_kinematics(
                frame_WCF, options={"solver": "ur5", "check_collision": True, "keep_order": False}
            )
        )
        assert len(solutions) == 6


@pytest.fixture
def frames_WCF():
    """A list of frames in the world coordinate frame for planning tests"""
    return [
        Frame((0.407, 0.073, 0.320), (0.922, 0.000, 0.388), (0.113, 0.956, -0.269)),
        Frame((0.404, 0.057, 0.324), (0.919, 0.000, 0.394), (0.090, 0.974, -0.210)),
        Frame((0.390, 0.064, 0.315), (0.891, 0.000, 0.454), (0.116, 0.967, -0.228)),
        Frame((0.388, 0.079, 0.309), (0.881, 0.000, 0.473), (0.149, 0.949, -0.278)),
        Frame((0.376, 0.087, 0.299), (0.850, 0.000, 0.528), (0.184, 0.937, -0.296)),
    ]


@pytest.fixture
def frame_waypoints(frames_WCF):
    """A FrameWaypoints Object for planning tests"""
    return FrameWaypoints(frames_WCF)


def test_kinematics_cartesian(frame_waypoints):
    if compas.IPY:
        return

    with AnalyticalPyBulletClient(connection_type="direct") as client:
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)

        options = {"solver": "ur5", "check_collision": True}

        trajectory = robot.plan_cartesian_motion(frame_waypoints, options=options)

        # Assert that the trajectory is complete
        assert trajectory.fraction == 1.0
        # Assert that the trajectory has the correct number of points
        assert len(trajectory.points) == len(frame_waypoints.target_frames)


def test_kinematics_cartesian_with_tool_coordinate_frame(frame_waypoints):
    if compas.IPY:
        return
    frame_waypoints.tool_coordinate_frame = Frame([0.01, 0.02, -0.03], [1, 0, 0], [0, 1, 0])

    with AnalyticalPyBulletClient(connection_type="direct") as client:
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)

        options = {"solver": "ur5", "check_collision": True}

        trajectory = robot.plan_cartesian_motion(frame_waypoints, options=options)

        # Assert that the trajectory is complete
        assert trajectory.fraction == 1.0
        # Assert that the trajectory has the correct number of points
        # NOTE: At the moment the AnalyticalPyBulletClient does not perform any interpolation between frames
        # NOTE: if future implementation fixes this, the following test will not be valid anymore
        assert len(trajectory.points) == len(frame_waypoints.target_frames)

import compas_fab
from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellState

with PyBulletClient() as client:
    urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
    robot = client.load_robot(urdf_filename)
    planner = PyBulletPlanner(client)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF)

    start_configuration = robot.zero_configuration()
    start_state = RobotCellState.from_robot_configuration(robot, start_configuration)

    configuration = planner.inverse_kinematics(target, start_state)

    print("Found configuration", configuration)

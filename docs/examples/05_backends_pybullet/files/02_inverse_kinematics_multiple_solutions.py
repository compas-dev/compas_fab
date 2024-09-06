from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary
from compas_fab.robots import TargetMode

with PyBulletClient() as client:
    # This example uses the panda robot, which has 7 joints.
    # When given a FrameTarget, the robot will have multiple inverse kinematics solutions.
    robot = RobotLibrary.panda()

    # Set RobotCell using the planner function
    robot_cell = RobotCell(robot)
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # Create initial state
    start_configuration = robot.zero_configuration()
    robot_cell_state = RobotCellState.from_robot_configuration(robot, start_configuration)

    # Create target
    frame_WCF = Frame([0.5, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    # The following demonstration shows that calling inverse_kinematics()
    # multiple times with exactly the same input will return different solutions
    # This can also be achieved by calling the function iter_inverse_kinematics()
    while True:
        config = planner.inverse_kinematics(target, robot_cell_state, options={"verbose": True})
        if config is None:
            break
        print("Inverse kinematics result: ", config.joint_values)
        input("Observe the IK result in PyBullet's GUI, Press Enter to find next solution...")

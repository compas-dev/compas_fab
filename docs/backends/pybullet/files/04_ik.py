from compas.geometry import Frame

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with PyBulletClient() as client:
    # Create a robot cell with a UR5 robot
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()

    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # The FrameTarget represents the robot's planner coordinate frame (PCF)
    # For the UR5 robot, the PCF is equal to the frame of the 'tool0' link
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    config = planner.inverse_kinematics(target, robot_cell_state)

    print("Inverse kinematics result: ", config)

    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # To verify the IK result, we can compute the FK with the obtained joint positions
    robot_cell_state.robot_configuration.merge(config)
    frame_WCF = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)
    print("Forward kinematics result (main group): \n ", frame_WCF)

    # The result is the same as the 'tool0' link's frame
    frame_WCF = planner.forward_kinematics_to_link(robot_cell_state, "tool0")
    print("Forward kinematics result: (tool0 link): \n ", frame_WCF)

    # However, note that the 'flange' link's frame has a different orientation
    frame_WCF = planner.forward_kinematics_to_link(robot_cell_state, "flange")
    print("Forward kinematics result: (flange link): \n ", frame_WCF)

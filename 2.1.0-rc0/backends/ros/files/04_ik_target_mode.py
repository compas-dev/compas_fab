from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Load robot from moveit to check the robot model name
    native_robot_cell = client.load_robot_cell()

    # Load existing robot cell from the library
    robot_cell, start_state = RobotCellLibrary.ur5_gripper_one_beam()
    assert robot_cell.robot_model.name == native_robot_cell.robot_model.name == "ur5_robot"
    planner.set_robot_cell(robot_cell)

    # Target using Planner Coordinate Frame (PCF)
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    print("\nTarget: {}".format(target))
    configuration = planner.inverse_kinematics(target, start_state)
    print("Configuration: {} ".format(configuration))

    # Target using Tool Coordinate Frame (TCF)
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.TOOL)
    print("\nTarget: {}".format(target))
    configuration = planner.inverse_kinematics(target, start_state)
    print("Configuration: {} ".format(configuration))

    # Target using Workpiece Object Coordinate Frame (OCF)
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.WORKPIECE)
    print("\nTarget: {}".format(target))
    configuration = planner.inverse_kinematics(target, start_state)
    print("Configuration: {} ".format(configuration))

    # Notice the difference in the resulting configurations

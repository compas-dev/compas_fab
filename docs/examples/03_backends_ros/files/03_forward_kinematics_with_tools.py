from compas.geometry import Frame
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode
from compas_fab.robots import RobotCellLibrary
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Loading the robot cell with tool and workpiece from the RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_gripper_one_beam(client)

    # Check that the robot in the ROS MoveIt backend is the same as the one in the RobotCellLibrary
    client.load_robot_cell(False)
    assert client.robot_cell.root_name == robot_cell.root_name

    # Set the tool and workpiece in the ROS MoveIt backend
    planner.set_robot_cell(robot_cell)

    # Modify the default robot_cell_state to change the robot's configuration and the attachment location of the beam
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]
    robot_cell_state.rigid_body_states["beam"].attachment_frame = Frame(
        [0.0, 0.0, -0.1], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
    )

    print("Robot Planner Coordinate Frame (PCF) relative to the world coordinate system (WCF):")
    frame = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)
    print("- {}".format(frame))

    print("Tool Coordinate Frame (TCF) relative to the world coordinate system (WCF):")
    frame = planner.forward_kinematics(robot_cell_state, TargetMode.TOOL)
    print("- {}".format(frame))

    print("Workpiece's Object Coordinate Frame (OCF) relative to the world coordinate system (WCF):")
    frame = planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)
    print("- {}".format(frame))

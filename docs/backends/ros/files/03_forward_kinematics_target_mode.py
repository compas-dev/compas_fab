from compas.geometry import Frame
from compas_fab.robots import RobotCellState
from compas_fab.robots import TargetMode
from compas_fab.robots import RobotCellLibrary
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Loading RobotCell from RobotCellLibrary with tool and workpiece
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_gripper_one_beam(client)

    # Load RobotCell from ROS MoveIt backend and assert that the robot model is the same
    client.load_robot_cell(False)
    assert client.robot_cell.root_name == robot_cell.root_name

    # Set the tool and workpiece in the ROS MoveIt backend
    planner.set_robot_cell(robot_cell)

    # The default robot cell state already have the gripper and beam attached
    # We modify the robot's configuration here to a specific joint configuration
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]
    robot_cell_state.rigid_body_states["beam"].attachment_frame = Frame(
        [0.0, 0.0, -0.1], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
    )

    # Note that the values for these frames are not the same.
    print("Robot Planner Coordinate Frame (PCF) relative to the world coordinate system (WCF):")
    frame = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)
    print("- {}".format(frame))

    print("Tool Coordinate Frame (TCF) relative to the world coordinate system (WCF):")
    frame = planner.forward_kinematics(robot_cell_state, TargetMode.TOOL)
    print("- {}".format(frame))

    print("Workpiece's Object Coordinate Frame (OCF) relative to the world coordinate system (WCF):")
    frame = planner.forward_kinematics(robot_cell_state, TargetMode.WORKPIECE)
    print("- {}".format(frame))

"""
Output:

>>> Robot Planner Coordinate Frame (PCF) relative to the world coordinate system (WCF):
>>> - Frame(point=Point(x=0.300, y=0.100, z=0.500), xaxis=Vector(x=-0.000, y=-1.000, z=0.000), yaxis=Vector(x=-0.000, y=-0.000, z=-1.000))
>>> Tool Coordinate Frame (TCF) relative to the world coordinate system (WCF):
>>> - Frame(point=Point(x=0.350, y=0.100, z=0.500), xaxis=Vector(x=1.000, y=-0.000, z=-0.000), yaxis=Vector(x=-0.000, y=-1.000, z=0.000))
>>> Workpiece's Object Coordinate Frame (OCF) relative to the world coordinate system (WCF):
>>> - Frame(point=Point(x=0.350, y=0.100, z=0.600), xaxis=Vector(x=1.000, y=-0.000, z=-0.000), yaxis=Vector(x=-0.000, y=-1.000, z=0.000))
"""

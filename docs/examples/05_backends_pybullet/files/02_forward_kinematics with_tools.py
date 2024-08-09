from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotLibrary
from compas.geometry import Frame
from compas.geometry import Box
from compas_robots import Configuration
from compas_robots import ToolModel

# Starting the PyBulletClient with the "direct" mode means that the GUI is not shown
with PyBulletClient("direct") as client:

    # Load a pre-made robot cell with one tool from the RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()

    # The planner is used for passing the robot cell into the PyBullet client
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # ---------------------
    # Compute FK with tools
    # ---------------------
    # The input configuration used for the forward kinematics is provided through the RobotCellState
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]
    # By default, if a tool is attached, the TCF is returned
    print("Frame of the attached tool TCF in World Coordinate Frame:")
    frame_WCF = planner.forward_kinematics(robot_cell_state)
    print(frame_WCF)

    # It is possible to retrieve T0CF by requesting with the end effector link name
    ee_link_name = robot_cell.robot.get_end_effector_link_name()
    print(f"Frame of the T0CF (name='{ee_link_name}') in World Coordinate Frame:")
    frame_WCF = planner.forward_kinematics(robot_cell_state, options={"link": ee_link_name})
    print(frame_WCF)

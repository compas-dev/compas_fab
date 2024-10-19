from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode
from compas_fab.robots import RobotCellState
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotCellLibrary
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

    # To retrieve the tool frame, the TargetMode.TOOL is used
    print("Frame of the attached tool TCF in World Coordinate Frame:")
    print(planner.forward_kinematics(robot_cell_state, TargetMode.TOOL))

    # To retrieve the Planner Coordinate Frame, the TargetMode.ROBOT is used
    print("Frame of the Planner Coordinate Frame PCF in World Coordinate Frame:")
    print(planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT))

    # It is also possible to retrieve the PCF  by requesting with the end effector link name
    ee_link_name = robot_cell.robot.get_end_effector_link_name()
    print("Frame of the T0CF (name='{}') in World Coordinate Frame:".format(ee_link_name))
    print(planner.forward_kinematics_to_link(robot_cell_state, ee_link_name))

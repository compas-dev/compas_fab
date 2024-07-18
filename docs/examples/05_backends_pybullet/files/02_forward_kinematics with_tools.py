from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotLibrary
from compas.geometry import Frame
from compas.geometry import Box
from compas_robots import Configuration
from compas_robots import ToolModel

with PyBulletClient() as client:
    # The robot in this example is loaded from the RobotLibrary
    robot = RobotLibrary.abb_irb4600_40_255()
    robot = client.load_existing_robot(robot)

    # ---------------------------------------------------------------------
    # Create a robot cell and add objects to it
    # ---------------------------------------------------------------------
    robot_cell = RobotCell(robot)

    # Add Static Collision Geometry
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody(floor_mesh)

    # Add Tool
    tool_mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
    tool_frame = Frame([0, 0, 0.14], [1, 0, 0], [0, 1, 0])
    robot_cell.tool_models["cone"] = ToolModel(tool_mesh, tool_frame)

    # Add workpiece at tool tip
    workpiece_mesh = Box(1.0, 0.1, 0.2).to_mesh(triangulated=True)
    robot_cell.rigid_body_models["workpiece"] = RigidBody(workpiece_mesh)

    # ------------------------------------------------------------------------
    # Create a RobotCellState to represent the current state of the robot cell
    # ------------------------------------------------------------------------
    robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

    # Attach the tool to the robot's main group
    robot_cell_state.set_tool_attached_to_group("cone", robot.main_group_name)

    # Attach the workpiece to the tool
    workpiece_grasp_frame = Frame([0, 0, 0.1], [1, 0, 0], [0, 1, 0])
    robot_cell_state.set_rigid_body_attached_to_tool("workpiece", "cone", workpiece_grasp_frame)

    # The planner is used for passing the robot cell into the PyBullet client
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # ----------------
    # FK without tools
    # ----------------

    # The input configuration used for the forward kinematics is provided through the RobotCellState
    robot_cell_state.robot_configuration = Configuration.from_revolute_values(
        [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]
    )
    # By default, if a tool is attached, the TCF is returned
    print("Frame of the attached tool TCF in World Coordinate Frame:")
    frame_WCF = planner.forward_kinematics(robot_cell_state)
    print(frame_WCF)

    # It is possible to retrieve T0CF by requesting with the end effector link name
    ee_link_name = robot.get_end_effector_link_name()
    print(f"Frame of the T0CF (also the end effector link {ee_link_name}) in World Coordinate Frame:")
    frame_WCF = planner.forward_kinematics(robot_cell_state, options={"link": ee_link_name})
    print(frame_WCF)

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
from compas_robots import ToolModel

with PyBulletClient() as client:
    # ---------------------------------------------------------------------
    # Create a robot cell and add objects to it
    # ---------------------------------------------------------------------
    robot = RobotLibrary.abb_irb4600_40_255()
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

    # Change the robot's configuration for demonstration purposes
    configuration = robot.zero_configuration()
    configuration.joint_values[1] = 0.5  # Change the second joint angle to 0.5 [rad]
    robot_cell_state.robot_configuration = configuration

    # Attach the tool to the robot's main group
    robot_cell_state.set_tool_attached_to_group("cone", robot.main_group_name)

    # Attach the workpiece to the tool
    workpiece_grasp_frame = Frame([0, 0, 0.1], [1, 0, 0], [0, 1, 0])
    robot_cell_state.set_rigid_body_attached_to_tool("workpiece", "cone", workpiece_grasp_frame)

    # The planner is used for passing the robot cell into the PyBullet client
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)  # or planner.set_robot_cell(robot_cell, robot_cell_state)
    planner.set_robot_cell_state(robot_cell_state)

    # ------------------------------------------------------------------------
    # Change the robot_cell_state and observe the effect in the PyBullet GUI
    # ------------------------------------------------------------------------

    # Typically the robot_cell_state is passed to the
    # planning functions such as planner.plan_motion(start_state, target), or
    # visualization functions such as robot_cell_scene_object.update(robot_cell_state).
    # In this example, we are directly calling set_robot_cell_state() to see the effect,
    # which can be seen in the PyBullet GUI.

    input("Observe the PyBullet GUI, Press Enter to continue...")

    for i in range(10):
        robot_cell_state.robot_configuration.joint_values[1] += 0.1
        planner.set_robot_cell_state(robot_cell_state)
        input("Observe the PyBullet GUI, Press Enter to continue...")

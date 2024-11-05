from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Box

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import ToolLibrary
from compas_fab.robots import RigidBody

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)

    # =========
    # Example 1
    # =========

    # Create a robot cell with the robot from the client
    # Add a floor as rigid body to the robot cell, this will be static.
    floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)
    # Add a demo cone tool to the robot cell, attachment is set later
    cone_tool = ToolLibrary.cone(radius=0.1, length=0.3)
    robot_cell.tool_models[cone_tool.name] = cone_tool
    # Set the robot cell in the planner
    result = planner.set_robot_cell(robot_cell)
    # If you are running ROS with UI, you should see a floor in the PyBullet world but the cone is not attached yet
    input("Press Enter to continue...")

    # In order to see the tool attached it is necessary to update the robot_cell_state
    robot_cell_state = robot_cell.default_cell_state()
    # Modify the tool state to attach the cone to the robot
    robot_cell_state.tool_states["cone"].attached_to_group = robot_cell.main_group_name
    robot_cell_state.tool_states["cone"].attachment_frame = Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    # Specify the link of the robot that the tool is allowed to collide with
    robot_cell_state.tool_states["cone"].touch_links = ["wrist_3_link"]
    # Move the robot to a different configuration
    robot_cell_state.robot_configuration._joint_values[1] = -0.5
    robot_cell_state.robot_configuration._joint_values[2] = 0.5
    # Specify the base of the robot is allowed to collide with the floor
    robot_cell_state.rigid_body_states["floor"].touch_links = ["base_link_inertia"]
    # Set the robot cell state in the planner
    result = planner.set_robot_cell_state(robot_cell_state)
    # If you are running ROS with UI, the cone should be attached to the robot
    input("Press Enter to continue...")

    # =========
    # Example 2
    # =========
    # Demonstrate that it is possible to have multiple tools in the robot cell

    # Create a robot cell with the robot from the client
    robot_cell = client.load_robot_cell()
    # Add a two tools (gripper and cone) to the robot cell
    gripper = ToolLibrary.static_gripper_small()
    robot_cell.tool_models[gripper.name] = gripper
    robot_cell.tool_models[cone_tool.name] = cone_tool
    # Attach the gripper to the robot
    robot_cell_state = robot_cell.default_cell_state()
    # The following function `set_tool_attached_to_group` is an alternative way than that in example 1
    # It will also ensure that only one tool is attached to the specified group by removing others
    robot_cell_state.set_tool_attached_to_group(
        gripper.name,
        robot_cell.main_group_name,
        attachment_frame=Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        touch_links=["wrist_3_link"],
    )
    # Specify the location of the detached cone tool
    robot_cell_state.tool_states[cone_tool.name].frame = Frame([1.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    # Move the robot to a different configuration
    robot_cell_state.robot_configuration = robot_cell.zero_configuration()
    robot_cell_state.robot_configuration._joint_values[1] = -0.5
    robot_cell_state.robot_configuration._joint_values[2] = 0.5
    result = planner.set_robot_cell(robot_cell, robot_cell_state)
    # If you are running ROS with UI, you should see a floor in the PyBullet world
    input("Press Enter to continue...")

    # =========
    # Example 3
    # =========

    # It is possible to remove all tools and objects from the robot cell
    planner.reset_planning_scene()

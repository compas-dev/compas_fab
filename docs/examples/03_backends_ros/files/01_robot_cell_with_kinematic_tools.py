from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import ToolLibrary

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)

    # =========
    # Example 1
    # =========

    # Add a kinematic gripper tool to the robot cell
    gripper = ToolLibrary.kinematic_gripper()
    robot_cell.tool_models[gripper.name] = gripper
    # Attach the gripper to the robot
    robot_cell_state = RobotCellState.from_robot_cell(robot_cell)
    robot_cell_state.set_tool_attached_to_group(
        gripper.name,
        robot_cell.main_group_name,
        attachment_frame=Frame([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        touch_links=["wrist_3_link"],  # This is the link that the tool is attached to
    )

    # Move the robot to a different configuration
    robot_cell_state.robot_configuration._joint_values[1] = -1.0
    robot_cell_state.robot_configuration._joint_values[2] = 0.5
    result = planner.set_robot_cell(robot_cell, robot_cell_state)
    print(result)

    # If you are running ROS with UI, you should see the gripper attached to the robot
    input("Press Enter to continue...")

    # =========
    # Example 2
    # =========

    # Kinematic tools can be moved with it's Configuration
    # The gripper tool's zero_configuration is a closed gripper state at [0, 0]
    print(gripper.zero_configuration)
    # The gripper tool's open gripper state is at [0.025, 0.025]
    robot_cell_state.tool_states[gripper.name].configuration.joint_values = [0.025, 0.025]

    # Remember to call `set_robot_cell_state` to update the robot cell in the planner
    result = planner.set_robot_cell_state(robot_cell_state)
    print(result)

    # If you are running ROS with UI, you should see the gripper fingers in an opened state.
    input("Press Enter to continue...")

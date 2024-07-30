import compas_fab
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary


# #############################################
# Headless (no-GUI) forwards kinematics example
# #############################################

# 'direct' mode
with PyBulletClient("direct") as client:
    # The robot in this example is loaded from a URDF file
    robot = RobotLibrary.ur5()
    planner = PyBulletPlanner(client)

    # This is a simple robot cell with only the robot
    robot_cell = RobotCell(robot)
    planner.set_robot_cell(robot_cell)

    # Create the starting configuration
    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.0])
    # The `RobotCellState.from_robot_configuration` method can be used when the robot is the only element in the cell
    robot_cell_state = RobotCellState.from_robot_configuration(robot, configuration)
    # In this demo, the default planning group is used for the forward kinematics
    frame_WCF = planner.forward_kinematics(robot_cell_state)

# Set the start and goal frames
start_frame = Frame([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
goal_frame = Frame([1.0, 1.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])

# Create a planner
planner = client.get_planner()

# Plan a cartesian motion using the pybullet backend
trajectory = planner.plan_cartesian_motion(robot, start_frame, goal_frame)

# Execute the planned trajectory
client.execute_trajectory(robot, trajectory)

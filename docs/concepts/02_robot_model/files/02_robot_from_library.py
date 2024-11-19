from compas_robots.resources import GithubPackageMeshLoader
from compas_fab.robots import RobotCellLibrary

# Load robot from RobotCellLibrary
# RobotCellLibrary also contains .ur5(), .ur10e(), abb_irb120_3_58(), abb_irb4600_40_255(), .rfl(), .panda()
robot_cell, robot_cell_state = RobotCellLibrary.ur5()
robot_model = robot_cell.robot_model
robot_semantics = robot_cell.robot_semantics

print(robot_model)

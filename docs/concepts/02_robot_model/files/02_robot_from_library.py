from compas_fab.robots import RobotCellLibrary

# Load robot from RobotCellLibrary
# RobotCellLibrary also contains .ur3(), .ur3e(), .ur5e(), .ur10(), .ur10e(),
# .ur16e(), .staubli_tx2_60l(), .abb_irb4600_40_255(), .rfl(), and .panda().
robot_cell, robot_cell_state = RobotCellLibrary.ur5()
robot_model = robot_cell.robot_model
robot_semantics = robot_cell.robot_semantics

print(robot_model)

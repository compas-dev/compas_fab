from compas_robots.resources import GithubPackageMeshLoader
from compas_fab.robots import RobotLibrary

# Load robot from RobotLibrary
# RobotLibrary also contains .ur5(), .ur10e(), abb_irb120_3_58(), abb_irb4600_40_255(), .rfl(), .panda()
robot = RobotLibrary.ur5()
model = robot.model

# Also load geometry
print(model)

from compas.geometry import Frame
from compas_robots import Configuration
from compas_fab.robots import RobotCellLibrary
from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.solvers import UR5Kinematics

from compas_fab.robots import RobotCell
from compas_fab.robots import TargetMode
from compas_fab.robots import RobotCellState

# Not loading the robot's geometry because AnalyticalKinematicsPlanner does not use it for collision checking
robot_cell, robot_cell_state = RobotCellLibrary.ur5(load_geometry=False)

# The kinematics_solver must match the robot's kinematics
planner = AnalyticalKinematicsPlanner(UR5Kinematics())

# This is a simple robot cell with only the robot
planner.set_robot_cell(robot_cell)

# Configuration for FK calculation
configuration = Configuration.from_revolute_values([0.0, 4.8, 1.5, 1.1, 1.9, 3.1])
robot_cell_state.robot_configuration = configuration

# AnalyticalKinematicsPlanner.forward_kinematics(), do not support `planning_group` parameter, it must be left as None.
# The `link` option is also not supported and cannot be used.
frame_WCF = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)

print("Robot flange frame of the default planning group in the world coordinate system:")
print(frame_WCF)

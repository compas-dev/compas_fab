from compas.geometry import Frame
from compas_fab.robots import RobotLibrary
from compas_fab.robots import FrameTarget
from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.solvers import UR5Kinematics

from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

# Not loading the robot's geometry because AnalyticalKinematicsPlanner does not use it for collision checking
robot = RobotLibrary.ur5(load_geometry=False)

# The kinematics_solver must match the robot's kinematics
planner = AnalyticalKinematicsPlanner(UR5Kinematics())

# This is a simple robot cell with only the robot
robot_cell = RobotCell(robot)
planner.set_robot_cell(robot_cell)

# IK Target
frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))  # Frame(point, xaxis, yaxis)
target = FrameTarget(frame_WCF)

# The start state is not important here because there is no tools involved
start_state = RobotCellState.from_robot_cell(robot_cell)

# iter_inverse_kinematics() will return a generator that would yield possible IK solutions
print("\nResults of iter_inverse_kinematics():")
for configuration in planner.iter_inverse_kinematics(target, start_state):
    # Note that although eight configurations are returned.
    # Some of the configurations may be in self-collision
    print(configuration)

# inverse_kinematics() will return each possible IK solutions one at a time.
print("\nResults of inverse_kinematics():")
for i in range(9):
    # Note that the last configuration is the same as the first one
    # because the robot has 8 possible solutions for this target
    # and calling the function repeatedly will return the same cycle of results
    configuration = planner.inverse_kinematics(target, start_state)
    print(configuration)

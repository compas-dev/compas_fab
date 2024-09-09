from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.geometry import Box

import compas_fab
from compas_fab.robots import RobotLibrary
from compas_fab.robots import FrameTarget
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyLibrary
from compas_fab.robots import TargetMode
from compas_robots import ToolModel

from compas_fab.backends import AnalyticalInverseKinematics
from compas_fab.backends import AnalyticalKinematicsPlanner
from compas_fab.backends.kinematics.solvers import UR5Kinematics

from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

# Not loading the robot's geometry because AnalyticalKinematicsPlanner does not use it for collision checking
robot = RobotLibrary.ur5(load_geometry=False)

# The kinematics_solver must match the robot's kinematics
planner = AnalyticalKinematicsPlanner(UR5Kinematics())

# ---------------------------------------------------------------------
# Create a robot cell and add objects to it
# ---------------------------------------------------------------------
robot_cell = RobotCell(robot)

# Add Static Collision Geometry
floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)

# Add Tool
tool_mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl"))
tool_frame = Frame([0, 0, 0.14], [1, 0, 0], [0, 1, 0])
robot_cell.tool_models["cone"] = ToolModel(tool_mesh, tool_frame)

# The robot cell is passed to the planner
planner.set_robot_cell(robot_cell)

# Create robot cell state, the default state from (.from_robot_cell) does not attach tools to the robot
robot_cell_state = RobotCellState.from_robot_cell(robot_cell)

# -----------------
# Define the target
# -----------------

# Create a Frame object: Frame(point, xaxis, yaxis)
target_frame = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))

# --------------------------------------
# First demonstrate the IK without tools
# --------------------------------------
# TODO: Confirm the following demo works after implementing AnalyticalInverseKinematics with TargetMode

# Create a target with TargetMode.ROBOT
target = FrameTarget(target_frame, TargetMode.ROBOT)
config = planner.inverse_kinematics(target, robot_cell_state)
print("IK Result (Configuration) without tools and the target represent T0CF:")
print(config)

# ------------------------------------
# Second demonstrate the IK with tools
# ------------------------------------
#  Modify the cell state to attach the tool to the robot
robot_cell_state.set_tool_attached_to_group("cone", robot.main_group_name)
# Create a target with TargetMode.TOOL
target = FrameTarget(target_frame, TargetMode.TOOL)
config = planner.inverse_kinematics(target, robot_cell_state)
print("IK Result (Configuration) with a tool attached and the target represent TCF of tool:")
print(config)

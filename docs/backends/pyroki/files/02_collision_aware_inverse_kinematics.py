from compas.datastructures import Mesh
from compas.geometry import Box

from compas_fab.backends import CollisionCheckError
from compas_fab.backends import PyRokiPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

robot_cell, state = RobotCellLibrary.panda(load_geometry=True)
state.robot_configuration.joint_values = [
    0.0,
    -0.785,
    0.0,
    -2.356,
    0.0,
    1.571,
    0.785,
    0.02,
]

# Keep the end-effector pose as the target and place an obstacle at the elbow.
# The redundant Panda arm can change posture while preserving that pose.
target_frame = robot_cell.robot_model.forward_kinematics(
    state.robot_configuration,
    robot_cell.get_end_effector_link_name(),
)
obstacle_frame = robot_cell.robot_model.forward_kinematics(
    state.robot_configuration,
    "panda_link4",
)
robot_cell.rigid_body_models["obstacle"] = RigidBody.from_mesh(Mesh.from_shape(Box(0.12)))
state.rigid_body_states["obstacle"] = RigidBodyState(obstacle_frame)

planner = PyRokiPlanner()
planner.set_robot_cell(robot_cell)

try:
    planner.check_collision(state, options={"full_report": True})
except CollisionCheckError as error:
    print("Starting collisions:", error.collision_pairs)

# Collision constraints are enabled by default.
configuration = planner.inverse_kinematics(
    FrameTarget(target_frame, TargetMode.ROBOT),
    state,
    options={"max_iterations": 200},
)
state.robot_configuration = state.robot_configuration.merged(configuration)
planner.check_collision(state)

actual_frame = robot_cell.robot_model.forward_kinematics(
    state.robot_configuration,
    robot_cell.get_end_effector_link_name(),
)
print(configuration)
print("Position error (m):", actual_frame.point.distance_to_point(target_frame.point))

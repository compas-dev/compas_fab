from compas_fab.backends import PyRokiPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

# Collision geometry is not needed for this kinematics-only example.
robot_cell, start_state = RobotCellLibrary.ur5(load_geometry=False)

# Build a reachable target from a known robot pose.
target_configuration = start_state.robot_configuration.copy()
target_configuration.joint_values = [0.3, -0.7, 0.8, -0.5, 0.6, -0.2]
target_frame = robot_cell.robot_model.forward_kinematics(
    target_configuration,
    robot_cell.get_end_effector_link_name(),
)

planner = PyRokiPlanner()
planner.set_robot_cell(robot_cell)
configuration = planner.inverse_kinematics(
    FrameTarget(target_frame, TargetMode.ROBOT),
    start_state,
    options={"check_collision": False},
)

# Verify the backend result with independent COMPAS Robots FK.
actual_frame = robot_cell.robot_model.forward_kinematics(
    configuration,
    robot_cell.get_end_effector_link_name(),
)
print(configuration)
print("Position error (m):", actual_frame.point.distance_to_point(target_frame.point))

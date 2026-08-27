from compas_fab.backends import PyRokiPlanner
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

robot_cell, state = RobotCellLibrary.ur5(load_geometry=False)
target_configuration = state.robot_configuration.copy()
target_configuration.joint_values = [0.4, -0.8, 0.9, -0.3, 0.7, 0.5]
target_frame = robot_cell.robot_model.forward_kinematics(
    target_configuration,
    robot_cell.get_end_effector_link_name(),
)

# The point and Z axis are constrained. Rotation about that Z axis remains free.
target = PointAxisTarget(
    target_frame.point,
    target_frame.zaxis,
    TargetMode.ROBOT,
)
planner = PyRokiPlanner()
planner.set_robot_cell(robot_cell)
configuration = planner.inverse_kinematics(
    target,
    state,
    options={"check_collision": False},
)

actual_frame = robot_cell.robot_model.forward_kinematics(
    configuration,
    robot_cell.get_end_effector_link_name(),
)
print(configuration)
print("Position error (m):", actual_frame.point.distance_to_point(target.target_point))
print("Axis error (rad):", actual_frame.zaxis.angle(target.target_z_axis))

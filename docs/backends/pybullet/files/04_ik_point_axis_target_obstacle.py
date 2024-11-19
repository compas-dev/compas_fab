# This example demonstrates the ability of PointAxisTarget to search a valid
# configuration around collision objects

from compas.geometry import Frame
from compas.geometry import Box
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import PointAxisTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RigidBody

with PyBulletClient() as client:

    # Load a pre-made robot cell with one tool from the RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_printing_tool()
    planner = PyBulletPlanner(client)

    # Add a collision object to the robot cell
    box = Box.from_corner_corner_height([0.0, 0.0, 0.0], [0.1, 0.2, 0.0], 2.0)
    rigid_body = RigidBody.from_mesh(box.to_mesh())
    robot_cell.rigid_body_models["box"] = rigid_body

    planner.set_robot_cell(robot_cell)

    # Create a PointAxisTarget to represents the tool's coordinate frame (TCF)
    target_center_point = [0.5, 2.0, 0.2]
    target_z_axis = [0, 1.0, -1.0]  # Tilted Axis
    target = PointAxisTarget(target_center_point, target_z_axis, TargetMode.TOOL)

    # Set the position of the box near the target
    robot_cell_state.rigid_body_states["box"] = RigidBodyState(
        Frame([0.15, 1.5, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    )

    # Run the IK function without collision checking, note the the robot's wrist is in collision with the box
    options = {"check_collision": False}
    config = planner.inverse_kinematics(target, robot_cell_state, options=options)
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # Run the IK function with collision checking, the planner will search for a different pose around the box
    options = {"check_collision": True}
    config = planner.inverse_kinematics(target, robot_cell_state, options=options)
    print("IK result with collision checking:", config)
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # Using iter_inverse_kinematics to explore all possible configurations around the box
    for config in planner.iter_inverse_kinematics(target, robot_cell_state, options=options):
        print("IK result:", config)
        input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

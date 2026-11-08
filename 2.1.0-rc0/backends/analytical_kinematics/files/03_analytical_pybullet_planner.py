# Demonstrates the AnalyticalPyBulletPlanner: closed-form analytical IK plus
# PyBullet collision checking, against a pre-configured robot cell with a tool
# attached and a stationary obstacle in the scene.

from compas.geometry import Box
from compas.geometry import Frame
from compas_robots import Configuration

from compas_fab.backends import ABB_IRB4600_40_255Kinematics
from compas_fab.backends import AnalyticalPyBulletClient
from compas_fab.backends import AnalyticalPyBulletPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with AnalyticalPyBulletClient(connection_type="gui") as client:
    # The library cell ships with the printing tool already attached and the
    # appropriate touch_links configured between the tool and the wrist.
    robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_printing_tool()
    planner = AnalyticalPyBulletPlanner(client, ABB_IRB4600_40_255Kinematics())

    # Add a static obstacle to the cell.
    obstacle_mesh = Box(0.2, 0.2, 1.0).to_mesh(triangulated=True)
    robot_cell.rigid_body_models["obstacle"] = RigidBody.from_mesh(obstacle_mesh)
    robot_cell_state.rigid_body_states["obstacle"] = RigidBodyState(
        Frame([1.0, -0.5, 0.5], [1, 0, 0], [0, 1, 0])
    )

    planner.set_robot_cell(robot_cell)

    # Derive a reachable target via forward kinematics from a chosen seed pose,
    # so we know at least some IK candidates exist near a known-good pose.
    seed = Configuration.from_revolute_values(
        [0.0, 0.3, 0.5, 0.0, 0.5, 0.0],
        joint_names=("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"),
    )
    robot_cell_state.robot_configuration.merge(seed)
    target_frame = planner.forward_kinematics(robot_cell_state, TargetMode.TOOL)
    print("Target frame (TCF, derived via FK):", target_frame)

    target = FrameTarget(target_frame, TargetMode.TOOL)

    # `keep_order=False` filters out any analytical candidate that would put
    # the robot, tool or workpiece in collision; only collision-free
    # configurations are yielded.
    options = {"check_collision": True, "keep_order": False}
    for i, config in enumerate(planner.iter_inverse_kinematics(target, robot_cell_state, options=options)):
        print(f"[{i}] {config}")
        # Visualize this configuration in the PyBullet GUI
        result_state = robot_cell_state.copy()  # type: RobotCellState
        result_state.robot_configuration = config
        planner.set_robot_cell_state(result_state)
        input("Press Enter to continue...")

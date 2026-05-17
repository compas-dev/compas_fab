from compas_robots import Configuration

from compas_fab.backends import AnalyticalPyBulletPlanner
from compas_fab.backends import PyBulletClient
from compas_fab.backends import UR5Kinematics
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with PyBulletClient(connection_type="direct") as client:
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()

    planner = AnalyticalPyBulletPlanner(client, UR5Kinematics())
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)

    # Pick a reachable, collision-free starting pose and use forward kinematics
    # to derive a target frame from it. This guarantees the target is reachable
    # by at least one configuration of this robot.
    seed = Configuration.from_revolute_values(
        [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
        joint_names=("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"),
    )
    robot_cell_state.robot_configuration.merge(seed)
    frame_WCF = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)
    print("Target frame (derived via FK):", frame_WCF)

    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    # `keep_order=True` returns one entry per analytical IK candidate, with
    # configurations that fail collision checking returned as `None` rather
    # than removed. The indices stay stable across calls, which is useful when
    # feeding adjacent IK calls into a Cartesian path solver.
    options = {"check_collision": True, "keep_order": True}

    for i, config in enumerate(planner.iter_inverse_kinematics(target, robot_cell_state, options=options)):
        if config is None:
            print(f"[{i}] (in collision)")
            continue
        print(f"[{i}] {config}")

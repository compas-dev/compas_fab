import compas_fab

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Box
from compas.geometry import Vector
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState

from _pybullet_demo_helper import trajectory_replay

with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)

    # The robot cell in this example is loaded from RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_printing_tool()

    box = Box.from_corner_corner_height([1.0, 1.0, 0], [2.0, 2.0, 0], 0.50)
    rigidbody = RigidBody(box.to_mesh(True), None)
    robot_cell.rigid_body_models["box"] = rigidbody
    robot_cell_state.rigid_body_states["box"] = RigidBodyState(Frame.worldXY())

    planner.set_robot_cell(robot_cell)
    # --------------------------------------
    # -------
    # Plan Cartesian Motion with FrameWaypoints
    # ---------------------------------------------

    # Perform IK to get the initial configuration
    first_target = PointAxisTarget(Point(1.0, 1.0, 0.5), Vector(0.5, 0.5, -1.0), TargetMode.TOOL)
    initial_configuration = planner.inverse_kinematics(first_target, robot_cell_state)

    # FrameWaypoints can hold more than one target frame
    points_and_axes = []
    # Move to X direction and move back
    points_and_axes.append((Point(2.0, 1.0, 0.5), Vector(-0.5, 0.5, -1.0)))
    points_and_axes.append((Point(2.0, 2.0, 0.5), Vector(-0.5, -0.5, -1.0)))
    points_and_axes.append((Point(1.0, 2.0, 0.5), Vector(0.5, -0.5, -1.0)))
    points_and_axes.append((Point(1.0, 1.0, 0.5), Vector(0.5, 0.5, -1.0)))
    # Move to Y direction and move back
    # points_and_axes.append((Point(0.4, 0.3, 0.5), Vector(0.0, 0.0, 1.0)))
    # points_and_axes.append((Point(0.4, 0.1, 0.5), Vector(0.0, 0.0, 1.0)))
    # # Move to Z direction and move back
    # points_and_axes.append((Point(0.4, 0.1, 0.7), Vector(0.0, 0.0, 1.0)))
    # points_and_axes.append((Point(0.4, 0.1, 0.5), Vector(0.0, 0.0, 1.0)))
    waypoints = PointAxisWaypoints(points_and_axes, target_mode=TargetMode.TOOL)

    print("initial_target:", first_target)
    print("initial_configuration:", initial_configuration)
    input("Press Enter to plan the trajectory...")

    # In this demo, the default planning group is used for the forward kinematics
    robot_cell_state.robot_configuration = initial_configuration
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state)

    print("Planned trajectory has {} points.".format(len(trajectory.points)))
    for i, point in enumerate(trajectory.points):
        print("- JointTrajectoryPoint {}, joint_values: {}".format(i, point.joint_values))
        robot_cell_state.robot_configuration = point
        frame = planner.forward_kinematics(robot_cell_state)
        print("  - Frame: {}".format(frame))

    # ------------------------------------------------
    # Replay the trajectory in the PyBullet simulation
    # ------------------------------------------------
    trajectory_replay(planner, robot_cell_state, trajectory)

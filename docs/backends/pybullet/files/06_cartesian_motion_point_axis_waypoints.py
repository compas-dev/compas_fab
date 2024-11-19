from _pybullet_demo_helper import trajectory_replay
from compas.geometry import Box
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector

from compas_fab.backends import MPNoIKSolutionError
from compas_fab.backends import MPNoPlanFoundError
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import PointAxisTarget
from compas_fab.robots import PointAxisWaypoints
from compas_fab.robots import RigidBody
from compas_fab.robots import RigidBodyState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)

    # The robot cell in this example is loaded from RobotCellLibrary
    # The printing tool TCP is defined with its Z axis pointing out of nozzle
    robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_printing_tool()

    # Add a box (without collision geometry) for demonstration visualization
    # Printing tool will trace a square around this box
    # box_size = 0.6 will demonstrate a successful planning
    # box_size = 0.8 will demonstrate a failure because it is out of reach
    box_size = 0.6
    box = Box.from_corner_corner_height([1.0, 1.0, 0], [1.0 + box_size, 1.0 + box_size, 0], 0.50)
    rigidbody = RigidBody(box.to_mesh(True), None)
    robot_cell.rigid_body_models["box"] = rigidbody
    robot_cell_state.rigid_body_states["box"] = RigidBodyState(Frame.worldXY())

    planner.set_robot_cell(robot_cell)
    # --------------------------------------
    # -------
    # Plan Cartesian Motion with FrameWaypoints
    # ---------------------------------------------

    # Perform IK to get the initial configuration - first corner of box
    first_target = PointAxisTarget(Point(1.0, 1.0, 0.5), Vector(0.5, 0.5, -1.0), TargetMode.TOOL)
    initial_configuration = planner.inverse_kinematics(first_target, robot_cell_state)

    # PointAxisWaypoints accepts a list of tuples, each containing a point and an axis
    points_and_axes = []
    # Move around in a square with some axis inclinations
    points_and_axes.append((Point(1.0 + box_size, 1.0, 0.5), Vector(-0.5, 0.5, -1.0)))
    points_and_axes.append((Point(1.0 + box_size, 1.0 + box_size, 0.5), Vector(-0.5, -0.5, -1.0)))
    points_and_axes.append((Point(1.0, 1.0 + box_size, 0.5), Vector(0.5, -0.5, -1.0)))
    points_and_axes.append((Point(1.0, 1.0, 0.5), Vector(0.5, 0.5, -1.0)))

    waypoints = PointAxisWaypoints(points_and_axes, target_mode=TargetMode.TOOL)

    print("initial_target:", first_target)
    print("initial_configuration:", initial_configuration)
    input("Press Enter to plan the trajectory...")

    # In this demo, the default planning group is used for the forward kinematics
    robot_cell_state.robot_configuration = initial_configuration
    try:
        trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state)
        print("Planned trajectory has {} points.".format(len(trajectory.points)))
    except MPNoIKSolutionError as e:
        # This exception is raised when part of the trajectory has no IK solution,
        # either due to collision or it is not reachable.
        print("No IK solution found. Reason:", e.message)
        print("Target that could not be reached:", e.target)
        trajectory = e.partial_trajectory
        print("Partial trajectory returned has {} points.".format(len(trajectory.points)))
    except MPNoPlanFoundError as e:
        # This exception is raised when no plan could be found, the IK solutions along the
        # trajectory are valid, but the planner could not find a continuous path between them.
        print("No plan found. Reason:", e.message)
        trajectory = e.partial_trajectory
        print("Partial trajectory returned has {} points.".format(len(trajectory.points)))

    # ------------------------------------------------
    # Replay the trajectory in the PyBullet simulation
    # ------------------------------------------------
    trajectory_replay(planner, robot_cell_state, trajectory)

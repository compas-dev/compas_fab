import compas_fab

from compas.geometry import Frame
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState

from _pybullet_demo_helper import trajectory_replay

with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)

    # The robot cell in this example is loaded from RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()
    planner.set_robot_cell(robot_cell)

    # The starting robot configuration is set in the robot cell state
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    # The following waypoint will cause the tool tip to rotate for 45 degrees and rotate back
    target_frames = []
    target_frames.append(Frame([0.4, 0.1, 0.5], [1.0, 1.0, 0.0], [1.0, -1.0, 0.0]))
    target_frames.append(Frame([0.4, 0.1, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]))
    waypoints = FrameWaypoints(target_frames)

    # -------------------------------------------------------
    # Example 1 - Plan Cartesian Motion with default settings
    # -------------------------------------------------------
    # The following trajectory will have very few points because rotation around Z axis has virtually no distance
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state)

    print("Planned trajectory with default settings has {} points.".format(len(trajectory.points)))

    # --------------------------------------------------------
    # Example 2 - Controlling max_step_angle
    # --------------------------------------------------------

    # Increasing the `max_step_angle` will cause the planner to have finer steps in the interpolation
    options = {"max_step_angle": 0.017}  # Roughly 1 degree
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options=options)
    # The frame is rotated by 90 degrees in total, so the trajectory should have roughly 90 points
    print("Planned trajectory with max_step_angle=0.01 has {} points.".format(len(trajectory.points)))

    # --------------------------------------------------------
    # Example 3 - Controlling max_jump_revolute
    # --------------------------------------------------------

    # An alternative is to increase the `max_jump_revolute` parameter
    # This will cause the planner to insert more points in the interpolation because the planned
    # configuration has too much distance to the next point

    options = {"max_jump_revolute": 0.087}  # Roughly 5 degrees
    # This means between each point, no revolute joint will move more than 5 degrees
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options=options)
    print("Planned trajectory with max_jump_revolute=0.1 has {} points.".format(len(trajectory.points)))

    # ------------------------------------------------
    # Replay the trajectory in the PyBullet simulation
    # ------------------------------------------------
    # The following code only serves demonstration purposes
    # User can step through the trajectory points by pressing 'Enter' key
    trajectory_replay(planner, robot_cell_state, trajectory)

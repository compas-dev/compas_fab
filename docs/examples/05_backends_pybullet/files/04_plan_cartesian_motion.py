import compas_fab

from compas.geometry import Frame
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import RobotCellState

with PyBulletClient("gui") as client:
    planner = PyBulletPlanner(client)

    # The robot cell in this example is loaded from RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_cone_tool()
    planner.set_robot_cell(robot_cell)

    # ---------------------------------------------
    # Plan Cartesian Motion with FrameWaypoints
    # ---------------------------------------------

    # The starting robot configuration is set in the robot cell state
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]
    # FrameWaypoints can hold more than one target frame
    target_frames = []
    # Move to X direction and move back
    target_frames.append(Frame([0.6, 0.1, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]))
    target_frames.append(Frame([0.4, 0.1, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]))
    # Move to Y direction and move back
    target_frames.append(Frame([0.4, 0.3, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]))
    target_frames.append(Frame([0.4, 0.1, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]))
    # Move to Z direction and move back
    target_frames.append(Frame([0.4, 0.1, 0.7], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]))
    target_frames.append(Frame([0.4, 0.1, 0.5], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]))
    waypoints = FrameWaypoints(target_frames)

    # In this demo, the default planning group is used for the forward kinematics
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state)

    print("Planned trajectory has {} points.".format(len(trajectory.points)))
    for i, point in enumerate(trajectory.points):
        print("- JointTrajectoryPoint {}, joint_values: {}".format(i, point.joint_values))

    # ------------------------------------------------
    # Replay the trajectory in the PyBullet simulation
    # ------------------------------------------------
    # The following code only serves demonstration purposes
    # User can step through the trajectory points by pressing 'Enter' key
    step = 0
    intermediate_robot_cell_state = robot_cell_state.copy()  # type: RobotCellState
    while True:
        if step >= len(trajectory.points):
            step = 0

        print("Step: {} - joint_values = {}".format(step, trajectory.points[step].joint_values))
        intermediate_robot_cell_state.robot_configuration = trajectory.points[step]
        planner.set_robot_cell_state(intermediate_robot_cell_state)

        input("Press Enter to continue...")
        step += 1

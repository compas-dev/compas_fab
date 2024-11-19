from compas.geometry import Frame

from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)
    robot_cell = client.load_robot_cell()
    robot_cell_state = robot_cell.default_cell_state()

    assert robot_cell.robot_model.name == "ur5_robot"

    # One straight segment
    frames = []
    frames.append(Frame([0.3, 0.3, 0.5], [0, -1, 0], [0, 0, -1]))
    waypoints = FrameWaypoints(frames, TargetMode.ROBOT)

    robot_cell_state.robot_configuration.joint_values = (-0.042, 0.033, -2.174, 5.282, -1.528, 0.000)

    options = {"max_step": 0.01}  # Units in meters
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options=options)
    print("Path (max_step=0.01) has %d configurations. " % len(trajectory.points))

    options = {"max_step": 0.001}  # Units in meters
    trajectory = planner.plan_cartesian_motion(waypoints, robot_cell_state, options=options)
    print("Path (max_step=0.001) has %d configurations. " % len(trajectory.points))


"""
Output: (may vary)
>>> Path (max_step=0.01) has 21 configurations.
>>> Path (max_step=0.001) has 201 configurations.
"""

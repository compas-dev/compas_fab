from compas.geometry import Box
from compas.geometry import Frame

from compas_fab.backends import MotionPlanningError
from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameWaypoints
from compas_fab.robots import RigidBody
from compas_fab.robots import TargetMode

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)
    assert robot_cell.robot_model.name == "ur5_robot"

    # Add an obstacle
    box_mesh = Box(0.1, 0.1, 1.2).to_mesh(triangulated=True)
    robot_cell.rigid_body_models["box"] = RigidBody.from_mesh(box_mesh)
    planner.set_robot_cell(robot_cell)

    # Define Starting State
    start_state = robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]
    # Set the obstacle's position
    start_state.rigid_body_states["box"].frame.point = [0.3, 0.0, 0.0]

    # Create waypoints
    frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
    waypoints = FrameWaypoints([frame], TargetMode.ROBOT)

    # This planning is expected to fail because it's impossible to follow the trajectory
    # in a linear motion, so the planner will fail but return a partial trajectory.
    try:
        trajectory = planner.plan_cartesian_motion(waypoints, start_state)
        print(f"Trajectory fraction: {trajectory.fraction}")
        print(f"Trajectory contains {len(trajectory.points)} configurations (JointTrajectoryPoint).")
        print(f"Executing this path at full speed would take approx. {trajectory.time_from_start} seconds.")
    except MotionPlanningError as e:
        print(f"Motion planning failed: {e}")
        print(f"Trajectory fraction: {e.partial_trajectory.fraction}")
        print(f"Trajectory contains {len(e.partial_trajectory.points)} configurations (JointTrajectoryPoint).")


"""
Output: (may vary)
>>> Motion planning failed: Motion planning failed
>>> Trajectory fraction: 0.14285714285714285
>>> Trajectory contains 12 configurations (JointTrajectoryPoint).
"""

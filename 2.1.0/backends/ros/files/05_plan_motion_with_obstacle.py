from compas.geometry import Box
from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
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

    # Create FrameTarget
    frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
    target = FrameTarget(frame, TargetMode.ROBOT)

    trajectory = planner.plan_motion(target, start_state)

    print("Trajectory contains %d configurations (JointTrajectoryPoint)." % len(trajectory.points))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

"""
Output: (may vary)
>>> Trajectory contains 79 configurations (JointTrajectoryPoint).
>>> Executing this path at full speed would take approx. 19.415 seconds.
"""

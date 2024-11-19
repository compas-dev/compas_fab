import math

from compas.geometry import Frame

from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode
from compas_fab.backends import MoveItPlanner

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)
    assert robot_cell.robot_model.name == "ur5_robot"

    # Define Starting State
    start_state = robot_cell.default_cell_state()
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]

    # Create FrameTarget
    frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
    target = FrameTarget(frame, TargetMode.ROBOT)

    trajectory = planner.plan_motion(target, start_state)

    print("Trajectory contains %d configurations (JointTrajectoryPoint)." % len(trajectory.points))
    print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

    # Print the JointTrajectoryPoint objects in the trajectory
    for point in trajectory.points:
        print(point)
"""
Output: (may vary)
>>> Trajectory contains 8 configurations (JointTrajectoryPoint).
>>> Executing this path at full speed would take approx. 2.377 seconds.
>>> JointTrajectoryPoint((-3.530, 3.830, -0.580, -3.330, 4.760, 0.000), (0, 0, 0, 0, 0, 0), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), (0.995, 0.000, 0.000, 0.000, 0.000, 0.000), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(0, 0))
>>> JointTrajectoryPoint((-3.330, 3.946, -0.723, -3.269, 4.903, -0.007), (0, 0, 0, 0, 0, 0), (0.531, 0.310, -0.381, 0.162, 0.379, -0.019), (0.957, 0.558, -0.687, 0.293, 0.684, -0.034), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(0, 633561529))
>>> JointTrajectoryPoint((-3.131, 4.063, -0.866, -3.208, 5.045, -0.014), (0, 0, 0, 0, 0, 0), (0.858, 0.500, -0.616, 0.262, 0.613, -0.031), (0.945, 0.551, -0.678, 0.289, 0.675, -0.034), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(0, 901041171))
>>> JointTrajectoryPoint((-2.931, 4.179, -1.010, -3.147, 5.188, -0.021), (0, 0, 0, 0, 0, 0), (1.054, 0.615, -0.756, 0.322, 0.753, -0.037), (0.884, 0.515, -0.634, 0.270, 0.631, -0.031), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 106861057))
>>> JointTrajectoryPoint((-2.731, 4.296, -1.153, -3.086, 5.330, -0.028), (0, 0, 0, 0, 0, 0), (1.059, 0.617, -0.759, 0.324, 0.756, -0.038), (-0.839, -0.489, 0.602, -0.256, -0.599, 0.030), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 282238794))
>>> JointTrajectoryPoint((-2.532, 4.412, -1.296, -3.025, 5.473, -0.035), (0, 0, 0, 0, 0, 0), (0.872, 0.508, -0.625, 0.266, 0.622, -0.031), (-0.925, -0.539, 0.663, -0.283, -0.660, 0.033), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 486109291))
>>> JointTrajectoryPoint((-2.332, 4.529, -1.439, -2.964, 5.615, -0.043), (0, 0, 0, 0, 0, 0), (0.541, 0.315, -0.388, 0.165, 0.386, -0.019), (-1.004, -0.585, 0.720, -0.307, -0.717, 0.036), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 747366937))
>>> JointTrajectoryPoint((-2.132, 4.645, -1.582, -2.903, 5.758, -0.050), (0, 0, 0, 0, 0, 0), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), (-1.008, -0.588, 0.723, -0.308, -0.719, 0.036), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(2, 376883245))
"""

import math

from compas.geometry import Frame

from compas_fab.backends import RosClient
from compas_fab.robots import ConfigurationTarget
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
    target_configuration = robot_cell.zero_full_configuration()
    target_configuration.joint_values = [-2.132, 4.645, -1.582, -2.903, 5.758, -0.050]
    target = ConfigurationTarget(target_configuration, tolerance_above=0.001, tolerance_below=0.001)

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
>>> JointTrajectoryPoint((-3.330, 3.946, -0.723, -3.269, 4.902, -0.007), (0, 0, 0, 0, 0, 0), (0.531, 0.310, -0.381, 0.162, 0.379, -0.019), (0.958, 0.558, -0.686, 0.292, 0.683, -0.035), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(0, 633592153))
>>> JointTrajectoryPoint((-3.130, 4.063, -0.866, -3.208, 5.045, -0.014), (0, 0, 0, 0, 0, 0), (0.859, 0.501, -0.615, 0.262, 0.612, -0.031), (0.944, 0.551, -0.677, 0.288, 0.674, -0.034), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(0, 901111462))
>>> JointTrajectoryPoint((-2.931, 4.179, -1.010, -3.147, 5.187, -0.022), (0, 0, 0, 0, 0, 0), (1.054, 0.615, -0.756, 0.322, 0.752, -0.038), (0.882, 0.514, -0.633, 0.269, 0.629, -0.032), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 106995939))
>>> JointTrajectoryPoint((-2.731, 4.296, -1.153, -3.086, 5.330, -0.029), (0, 0, 0, 0, 0, 0), (1.059, 0.618, -0.759, 0.323, 0.755, -0.038), (-0.838, -0.488, 0.600, -0.256, -0.597, 0.030), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 282458174))
>>> JointTrajectoryPoint((-2.531, 4.412, -1.296, -3.025, 5.472, -0.036), (0, 0, 0, 0, 0, 0), (0.872, 0.508, -0.625, 0.266, 0.622, -0.032), (-0.925, -0.539, 0.663, -0.282, -0.660, 0.034), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 486375649))
>>> JointTrajectoryPoint((-2.331, 4.529, -1.439, -2.964, 5.615, -0.043), (0, 0, 0, 0, 0, 0), (0.541, 0.315, -0.388, 0.165, 0.386, -0.020), (-1.004, -0.585, 0.720, -0.307, -0.716, 0.036), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(1, 747682667))
>>> JointTrajectoryPoint((-2.132, 4.645, -1.582, -2.903, 5.757, -0.051), (0, 0, 0, 0, 0, 0), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), (-1.008, -0.588, 0.723, -0.308, -0.719, 0.037), (0.000, 0.000, 0.000, 0.000, 0.000, 0.000), Duration(2, 377232469))
"""

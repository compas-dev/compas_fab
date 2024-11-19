from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

from compas_fab.robots import RobotCellLibrary

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Use a predefined robot cell with a gripper and a single beam
    robot_cell, start_state = RobotCellLibrary.ur5_gripper_one_beam()
    planner.set_robot_cell(robot_cell)

    # Define Starting Configuration
    start_state.robot_configuration.joint_values = [-3.53, 3.83, -0.58, -3.33, 4.76, 0.00]

    # Create FrameTarget for the workpiece
    target_frame = Frame([0.4, 0.1, 0.6], [1, 0, 0], [0, -1, 0])
    target = FrameTarget(target_frame, TargetMode.WORKPIECE)

    trajectory = planner.plan_motion(target, start_state)

    print("Trajectory contains %d configurations (JointTrajectoryPoint)." % len(trajectory.points))
    final_joint_values = trajectory.points[-1].joint_values
    print("   Joint values of the final trajectory point: {}".format(final_joint_values))

    # Validate the pose of the workpiece after planning
    end_state = start_state.copy()
    end_state.robot_configuration.joint_values = final_joint_values
    workpiece_frame = planner.forward_kinematics(end_state, TargetMode.WORKPIECE)

    print("Workpiece's Object Coordinate Frame (OCF): {}".format(workpiece_frame))
    planning_error = workpiece_frame.point.distance_to_point(target_frame.point)
    print("   Target distance error: {} (meter)".format(planning_error))

"""
Output: (may vary)
>>> Trajectory contains 11 configurations (JointTrajectoryPoint).
>>>    Joint values of the final trajectory point: [-2.4166840511621435, 4.937602986182445, -1.317040437371621, -3.5540095964662965, 5.338763175705421, -3.149330570899709]
>>> Workpiece's Object Coordinate Frame (OCF): Frame(point=Point(x=0.398, y=0.103, z=0.597), xaxis=Vector(x=0.994, y=0.097, z=0.054), yaxis=Vector(x=0.099, y=-0.995, z=-0.031))
>>>    Target distance error: 0.004860285596193909 (meter)
"""

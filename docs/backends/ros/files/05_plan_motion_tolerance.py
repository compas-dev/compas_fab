import math

from compas.geometry import Frame
from compas.geometry import axis_angle_from_quaternion
from compas.geometry import norm_vector


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

    # Plan motion and validate the final position error
    def plan_and_validate(target):
        # type: (FrameTarget) -> None
        trajectory = planner.plan_motion(target, start_state)
        print("Trajectory contains %d configurations (JointTrajectoryPoint)." % len(trajectory.points))
        end_state = start_state.copy()
        end_state.robot_configuration.joint_values = trajectory.points[-1].joint_values
        robot_frame = planner.forward_kinematics(end_state, TargetMode.ROBOT)
        print("   Final Pose: {}".format(robot_frame))
        position_error = target.target_frame.point.distance_to_point(robot_frame.point)
        print("   Distance error: {} (meter)".format(position_error))
        delta_frame = robot_frame.to_local_coordinates(target.target_frame)  # type: Frame
        _, orientation_error = axis_angle_from_quaternion(delta_frame.quaternion)
        print("   Orientation error: {} (radians)".format(orientation_error))

    # Create FrameTarget with default tolerance and plan
    target_frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
    target = FrameTarget(target_frame, TargetMode.ROBOT)
    plan_and_validate(target)

    # Set target tolerance to tighter values and plan again
    target.tolerance_position = 0.0001  # Unit is meters (default is 0.001)
    target.tolerance_orientation = 0.001  # Unit is radians  (default is 0.01)
    plan_and_validate(target)

"""
Output: (may vary)
>>> Trajectory contains 8 configurations (JointTrajectoryPoint).
>>>    Final Pose: Frame(point=Point(x=0.400, y=0.300, z=0.400), xaxis=Vector(x=-0.001, y=1.000, z=0.007), yaxis=Vector(x=-0.004, y=-0.007, z=1.000))
>>>    Distance error: 0.0005173194587181946 (meter)
>>>    Orientation error: 0.00808542020121989 (radians)
>>> Trajectory contains 8 configurations (JointTrajectoryPoint).
>>>    Final Pose: Frame(point=Point(x=0.400, y=0.300, z=0.400), xaxis=Vector(x=-0.000, y=1.000, z=0.001), yaxis=Vector(x=0.001, y=-0.001, z=1.000))
>>>    Distance error: 4.790417904127647e-05 (meter)
>>>    Orientation error: 0 (radians)
"""

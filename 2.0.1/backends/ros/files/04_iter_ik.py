from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Create a default RobotCellState from the RobotCell as the starting state
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    start_state = robot_cell.default_cell_state()

    # Create a target frame with the ROBOT mode
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    print("Target frame with Robot Mode: {}".format(target))

    # User iter inverse kinematics to find multiple solutions
    configuration_generator = planner.iter_inverse_kinematics(target, start_state)
    for i in range(10):
        configuration = next(configuration_generator)
        print("IK Solution {}: {}".format(i, configuration))

"""
Output:
>>> Target frame with Robot Mode: FrameTarget(Frame(point=Point(x=0.300, y=0.100, z=0.500), xaxis=Vector(x=1.000, y=0.000, z=0.000), yaxis=Vector(x=0.000, y=1.000, z=0.000)), ROBOT)
>>> IK Solution 0: Configuration((-0.031, -2.025, 2.161, 4.576, -4.712, -1.540), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 1: Configuration((3.816, 3.155, 2.161, -3.746, -1.571, -2.245), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 2: Configuration((-2.467, -1.117, -2.161, -1.434, -1.571, -2.245), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 3: Configuration((-0.031, -2.025, 2.161, -1.707, -4.712, -1.540), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 4: Configuration((-0.031, -6.135, -1.793, -3.068, -1.571, -4.682), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 5: Configuration((6.253, 0.148, -1.793, -3.068, 4.712, -4.682), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 6: Configuration((-0.031, -0.013, -2.161, 0.604, -4.712, -1.540), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 7: Configuration((-2.467, 5.167, -2.161, 4.849, 4.712, -2.245), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 8: Configuration((-0.031, 0.148, -1.793, 3.215, -1.571, 1.601), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> IK Solution 9: Configuration((3.816, -1.117, -2.161, 4.849, -1.571, -2.245), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
"""

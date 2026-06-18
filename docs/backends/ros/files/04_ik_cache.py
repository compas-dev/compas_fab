from compas.geometry import Frame

from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)

    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    start_state = robot_cell.default_cell_state()

    # Create a target frame with the ROBOT mode
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    options = {"max_results": 100}

    # First Call to Inverse Kinematics
    configuration = planner.inverse_kinematics(target, start_state, options=options)
    print("Configuration 1: ", configuration)

    # Subsequent Call to Inverse Kinematics with exactly the same input
    configuration = planner.inverse_kinematics(target, start_state, options=options)
    print("Configuration 2: ", configuration)
    configuration = planner.inverse_kinematics(target, start_state, options=options)
    print("Configuration 3: ", configuration)

"""
Output: (Second and third configuration is random)
>>> Configuration 1:  Configuration((-0.031, -2.025, 2.161, 4.576, -4.712, -1.540), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> Configuration 2:  Configuration((3.816, -1.117, -2.161, -1.434, 4.712, 4.038), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
>>> Configuration 3:  Configuration((-0.031, -0.013, -2.161, 0.604, 1.571, 4.743), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
"""

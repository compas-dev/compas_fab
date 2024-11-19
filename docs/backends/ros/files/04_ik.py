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

    # Calculate inverse kinematics
    configuration = planner.inverse_kinematics(target, start_state)
    print("Configuration 1: ", configuration)

    # Try a slightly different target frame
    target.target_frame.point.z += 0.1
    # Use the found configuration as the starting point for the second IK calculation
    start_state.robot_configuration.merge(configuration)
    configuration = planner.inverse_kinematics(target, start_state)
    print("Configuration 2: ", configuration)

"""
Output:
>>> Target frame with Robot Mode: FrameTarget(Frame(point=Point(x=0.300, y=0.100, z=0.500), xaxis=Vector(x=1.000, y=0.000, z=0.000), yaxis=Vector(x=0.000, y=1.000, z=0.000)), ROBOT)
>>> Configuration 1:  Configuration((-0.031, -2.025, 2.161, 4.576, -4.712, -1.540), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
>>> 'wrist_2_joint', 'wrist_3_joint'))
>>> Configuration 2:  Configuration((-0.031, -2.027, 1.907, 4.833, -4.712, -1.540), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
>>> 'wrist_2_joint', 'wrist_3_joint'))
"""

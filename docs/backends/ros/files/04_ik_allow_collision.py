from compas.geometry import Box
from compas.geometry import Frame

from compas_fab.backends import InverseKinematicsError
from compas_fab.backends import MoveItPlanner
from compas_fab.backends import RosClient
from compas_fab.robots import FrameTarget
from compas_fab.robots import RigidBody
from compas_fab.robots import TargetMode

with RosClient() as client:
    planner = MoveItPlanner(client)

    # Create a default RobotCellState from the RobotCell as the starting state
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"

    # Add a collision geometry to the robot cell
    box_mesh = Box(0.1, 0.1, 0.6).to_mesh(triangulated=True)
    robot_cell.rigid_body_models["box"] = RigidBody.from_mesh(box_mesh)
    start_state = robot_cell.default_cell_state()
    start_state.rigid_body_states["box"].frame.point = [0.3, 0.1, 0.3]
    planner.set_robot_cell(robot_cell, start_state)

    # Create a target frame with the ROBOT mode
    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    print("Target frame with Robot Mode: {}".format(target))

    # Calculate inverse kinematics
    def calculate(options):
        try:
            configuration = planner.inverse_kinematics(target, start_state, options=options)
            print("   Found configuration: ", configuration)
        except InverseKinematicsError as e:
            print("   Failed to find a configuration: ", e)

    print("Inverse kinematics with collision check enabled: (by default)")
    calculate({})

    print("Inverse kinematics with collision check disabled:")
    options = {"allow_collision": True}
    calculate(options)

"""
Output:
>>> Target frame with Robot Mode: FrameTarget(Frame(point=Point(x=0.300, y=0.100, z=0.500), xaxis=Vector(x=1.000, y=0.000, z=0.000), yaxis=Vector(x=0.000, y=1.000, z=0.000)), ROBOT)
>>> Inverse kinematics with collision check enabled: (by default)
>>>    Failed to find a configuration:  No inverse kinematics solution found.
>>> Inverse kinematics with collision check disabled:
>>>    Found configuration:  Configuration((-0.031, -2.025, 2.161, 4.576, -4.712, -1.540), (0, 0, 0, 0, 0, 0), ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))
"""

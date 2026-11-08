from compas.geometry import Frame

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with PyBulletClient() as client:
    # This example uses the panda robot, which has 7 joints.
    # When given a FrameTarget, the robot will have multiple inverse kinematics solutions.
    robot_cell, robot_cell_state = RobotCellLibrary.panda()

    # Set RobotCell using the planner function
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # Create target
    frame_WCF = Frame([0.5, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    # The following demonstration shows that calling inverse_kinematics()
    # multiple times with exactly the same input will return different solutions
    # This can also be achieved by calling the function iter_inverse_kinematics()
    while True:
        # Verbose is set to True to show the collision checking process
        config = planner.inverse_kinematics(target, robot_cell_state, options={"verbose": True})
        if config is None:
            break
        # Note that the result are all unique because a uniqueness filter is built-in to the planner
        # The uniqueness tolerance can be adjusted with the options parameter
        print("Inverse kinematics result: ", config.joint_values)
        input("Observe the IK result in PyBullet's GUI, Press Enter to find next solution...")

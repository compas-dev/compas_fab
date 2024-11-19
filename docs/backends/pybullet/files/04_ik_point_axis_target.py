from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import PointAxisTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

with PyBulletClient() as client:

    # Load a pre-made robot cell with one tool from the RobotCellLibrary
    robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_printing_tool()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)
    # Observe the starting configuration in PyBullet's GUI
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # Create a PointAxisTarget to represents the tool's coordinate frame (TCF)
    target_center_point = [0.5, 1.5, 0.2]
    target_z_axis = [0.2, 0.2, -1]  # Slightly tilted
    target = PointAxisTarget(target_center_point, target_z_axis, TargetMode.TOOL)

    # Options for planning with PointAxisTarget
    # See documentation of PyBulletInverseKinematics._iter_inverse_kinematics_point_axis_target() for more options
    options = {"num_rotation_steps": 20, "max_random_restart": 5}
    config = planner.inverse_kinematics(target, robot_cell_state, options=options)
    print("Inverse kinematics result: ", config)
    input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # The planner automatically plans such that the tool's tip is at the target's center point
    # The following `iter_` method will return all possible configurations that can reach the target
    # In PyBullet's GUI, notice the tool's tip stays at the same location, only the orientation changes
    total_results = 0
    for config in planner.iter_inverse_kinematics(target, robot_cell_state):
        total_results += 1
        print("Inverse kinematics result: ", config)
        input("Observe the IK result in PyBullet's GUI, Press Enter to continue...")

    # The total number of results is affected by many factors.
    # Firstly, the degree of freedom of the robot.
    # Secondly, the reachability of the target and the presence of collision objects limits the number of results.
    # Finally, planning options such as the number of random restarts, the number of rotation steps, etc.
    print("Total results: ", total_results)

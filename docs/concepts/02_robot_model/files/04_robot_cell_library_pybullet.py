from compas_fab.backends.pybullet import PyBulletClient
from compas_fab.backends.pybullet import PyBulletPlanner

from compas_fab.robots import RobotCellLibrary

with PyBulletClient() as client:
    planner = PyBulletPlanner(client)
    robot_cell, robot_cell_state = RobotCellLibrary.abb_irb4600_40_255_gripper_one_beam(client, load_geometry=True)
    planner.set_robot_cell(robot_cell, robot_cell_state)
    input("Press Enter to exit...")

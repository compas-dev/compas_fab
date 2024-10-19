import compas_fab
from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode


with PyBulletClient(connection_type="direct") as client:
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    target = FrameTarget(frame_WCF, TargetMode.ROBOT)

    start_configuration = robot_cell.zero_configuration()
    start_state = RobotCellState.from_robot_configuration(robot, start_configuration)

    options = {"max_results": 20}
    result_count = 0
    for config in planner.iter_inverse_kinematics(target, start_state, options=options):
        print("Found configuration", config)
        result_count += 1
    print("Found %d configurations" % result_count)

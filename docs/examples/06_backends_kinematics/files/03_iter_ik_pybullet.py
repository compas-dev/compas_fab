from compas.geometry import Frame
from compas_fab.backends import AnalyticalPyBulletPlanner
from compas_fab.backends import UR5Kinematics
from compas_fab.backends import PyBulletClient
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import FrameTarget
from compas_fab.robots import TargetMode

frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))


with PyBulletClient(connection_type="direct") as client:
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()

    kinematics = UR5Kinematics()
    planner = AnalyticalPyBulletPlanner(client, kinematics)

    options = {"check_collision": True, "keep_order": True}

    target = FrameTarget(frame_WCF, TargetMode.ROBOT)
    for config in planner.iter_inverse_kinematics(target, robot_cell_state, options=options):
        print(config)

from compas.geometry import Frame
from compas_fab.robots import RobotLibrary
from compas_fab.backends import AnalyticalInverseKinematics

ik = AnalyticalInverseKinematics()
robot = RobotLibrary.ur5()

frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))

# The solver must be one of the supported PLANNER_BACKENDS
for joint_positions, joint_names in ik.inverse_kinematics(robot, frame_WCF, options={'solver': 'ur5'}):
    print(joint_positions)

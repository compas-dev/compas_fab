from compas.geometry import Frame
from compas_fab.robots.ur5 import Robot
from compas_fab.backends import AnalyticalInverseKinematics

ik = AnalyticalInverseKinematics()
robot = Robot()

frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))

for jp, jn in ik.inverse_kinematics(robot, frame_WCF, options={'solver': 'ur5'}):  # knows that we need the IK for the UR5 robot
    print(jp)

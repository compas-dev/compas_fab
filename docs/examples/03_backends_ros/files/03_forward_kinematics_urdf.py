from compas.robots import Configuration
from compas_fab.robots.ur5 import Robot

robot = Robot()
configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.])

frame_WCF = robot.forward_kinematics(configuration)

print("Frame in the world coordinate system")
print(frame_WCF)

assert repr(frame_WCF.point) == 'Point(0.300, 0.100, 0.500)'

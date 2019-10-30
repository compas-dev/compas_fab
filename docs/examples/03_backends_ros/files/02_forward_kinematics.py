from compas_fab.robots import Configuration
from compas_fab.robots.ur5 import Robot

robot = Robot()
configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.])

frame_RCF = robot.forward_kinematics(configuration)
frame_WCF = robot.to_world_coords(frame_RCF)

print("Frame in the robot's coordinate system")
print(frame_RCF)
print("Frame in the world coordinate system")
print(frame_WCF)

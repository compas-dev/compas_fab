from compas_fab.backends import RosClient
from compas_fab.robots import Configuration
from compas_fab.robots.ur5 import Robot

with RosClient() as client:
    robot = Robot(client)
    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.])

    response = robot.forward_kinematics(configuration)

    print("Frame in the robot's coordinate system", response.frame_RCF)
    print("Frame in the world coordinate system", response.frame_WCF)

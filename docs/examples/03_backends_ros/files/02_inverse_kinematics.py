from compas.geometry import Frame
from compas_fab.backends import RosClient
from compas_fab.robots.ur5 import Robot

with RosClient() as client:
    robot = Robot(client)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    start_configuration = robot.zero_configuration()

    configuration = robot.inverse_kinematics(frame_WCF, start_configuration)

    print("Found configuration", configuration)

from compas.geometry import Frame
from compas_fab.backends import RosClient

with RosClient() as client:
    robot = client.load_robot()

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    start_configuration = robot.init_configuration()

    configuration = robot.inverse_kinematics(frame_WCF, start_configuration)

    print("Found configuration", configuration)

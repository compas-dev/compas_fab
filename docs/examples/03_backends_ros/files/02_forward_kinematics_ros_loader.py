from compas_fab.backends import RosClient
from compas_fab.robots import Configuration

with RosClient() as client:
    robot = client.load_robot()

    configuration = Configuration.from_prismatic_and_revolute_values([0.], [-2.238, -1.153, -2.174, 0.185, 0.667, 0.])

    frame_RCF = robot.forward_kinematics(configuration)
    frame_WCF = robot.to_world_coords(frame_RCF)

    print("Frame in the robot's coordinate system")
    print(frame_RCF)
    print("Frame in the world coordinate system")
    print(frame_WCF)

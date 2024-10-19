from compas_robots import Configuration
from compas_fab.backends import RosClient

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    print(robot_cell.robot_model.name)
    assert robot.name == "ur5_robot"

    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.0])

    frame_WCF = robot.forward_kinematics(configuration)

    print("Frame in the world coordinate system")
    print(frame_WCF)

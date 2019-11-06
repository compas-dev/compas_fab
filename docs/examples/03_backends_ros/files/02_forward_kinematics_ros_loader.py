from compas.robots import RobotModel

from compas_fab.backends import RosClient
from compas_fab.backends import RosFileServerLoader
from compas_fab.robots import Configuration
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics

with RosClient() as client:
    loader = RosFileServerLoader(client)

    urdf = loader.load_urdf()
    srdf = loader.load_srdf()

    model = RobotModel.from_urdf_string(urdf)
    semantics = RobotSemantics.from_srdf_string(srdf, model)

    robot = Robot(model, semantics=semantics, client=client)

    configuration = Configuration.from_prismatic_and_revolute_values([0.], [-2.238, -1.153, -2.174, 0.185, 0.667, 0.])

    frame_RCF = robot.forward_kinematics(configuration)
    frame_WCF = robot.to_world_coords(frame_RCF)

    print("Frame in the robot's coordinate system")
    print(frame_RCF)
    print("Frame in the world coordinate system")
    print(frame_WCF)

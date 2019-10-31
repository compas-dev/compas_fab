from compas.geometry import Frame
from compas.robots import RobotModel

from compas_fab.backends import RosClient
from compas_fab.backends import RosFileServerLoader
from compas_fab.robots import Robot
from compas_fab.robots import RobotSemantics

with RosClient() as client:
    loader = RosFileServerLoader(client)

    urdf = loader.load_urdf()
    srdf = loader.load_srdf()

    model = RobotModel.from_urdf_string(urdf)
    semantics = RobotSemantics.from_srdf_string(srdf, model)

    robot = Robot(model, semantics=semantics, client=client)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    start_configuration = robot.init_configuration()

    configuration = robot.inverse_kinematics(frame_WCF, start_configuration)

    print("Found configuration", configuration)

from compas.robots import RobotModel

from compas_fab.backends import RosClient
from compas_fab.robots import Robot

with RosClient() as client:
    model = RobotModel.ur5()
    robot = Robot(model, client=client)

    robot.info()

    assert len(robot.get_configurable_joint_names()) == 6

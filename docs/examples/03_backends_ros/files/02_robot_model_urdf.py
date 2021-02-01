from compas.robots import RobotModel

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import Robot

with RosClient() as client:
    urdf = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    model = RobotModel.from_urdf_file(urdf)
    robot = Robot(model, client=client)

    robot.info()

    assert robot.name == 'ur5'

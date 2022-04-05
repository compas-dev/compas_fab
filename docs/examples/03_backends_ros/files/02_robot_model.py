from compas_fab.backends import RosClient

with RosClient() as client:
    robot = client.load_robot()
    robot.info()

    assert robot.name == 'ur5_robot'

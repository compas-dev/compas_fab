from compas_fab.backends import RosClient

with RosClient() as ros:
    robot = ros.load_robot(load_geometry=True, precision=12)

    print(robot.model)

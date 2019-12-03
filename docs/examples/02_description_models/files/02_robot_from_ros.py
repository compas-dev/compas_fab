import compas
from compas_fab.backends import RosClient

# Set high precision to import meshes defined in meters
compas.PRECISION = '12f'

with RosClient() as ros:
    robot = ros.load_robot(load_geometry=True)

    print(robot.model)

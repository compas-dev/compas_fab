import compas
from compas_fab.backends import RosClient
from compas_rhino.artists import RobotModelArtist

# Set high precision to import meshes defined in meters
compas.PRECISION = '12f'

with RosClient() as ros:
    # Load complete model from ROS
    robot = ros.load_robot(load_geometry=True)

    # Visualize robot
    robot.artist = RobotModelArtist(robot.model, layer='COMPAS FAB::Example')
    robot.artist.clear_layer()
    robot.artist.draw_visual()

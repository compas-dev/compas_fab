from compas_fab.backends import RosClient
from compas_rhino.artists import RobotModelArtist

with RosClient() as ros:
    # Load complete model from ROS
    robot = ros.load_robot(load_geometry=True, precision=12)

    # Visualize robot
    robot.artist = RobotModelArtist(robot.model, layer="COMPAS FAB::Example")
    robot.artist.clear_layer()
    robot.artist.draw_visual()

import os

from compas_fab.backends import RosClient

with RosClient() as ros:
    # Load complete model from ROS and set a local cache location
    local_directory = os.path.join(os.path.expanduser("~"), "robot_description", "robot_name")
    robot_cell = ros.load_robot_cell(load_geometry=True, local_cache_directory=local_directory, precision=12)

    robot_cell.print_info()

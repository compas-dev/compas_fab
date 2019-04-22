import os

import compas
from compas.robots import RobotModel
from compas_fab.backends import RosClient
from compas_fab.robots import RobotSemantics
from compas_fab.robots import RosFileServerLoader

# Set high precision to import meshes defined in meters
compas.PRECISION = '12f'

with RosClient() as ros:
    # Load URDF from ROS with local cache enabled
    local_directory = os.path.join(os.path.expanduser('~'), 'robot_description')
    loader = RosFileServerLoader(ros, local_cache=True, local_cache_directory=local_directory)
    loader.robot_name = 'abb_irb1600_6_12'

    urdf = loader.load_urdf()

    # Create robot model from URDF and load geometry
    model = RobotModel.from_urdf_string(urdf)
    model.load_geometry(loader)

    print(model)

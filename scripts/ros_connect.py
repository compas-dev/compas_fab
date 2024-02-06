# This script extracts the URDF from a MoveIt! instance running in docker
import os

from compas_fab.backends.ros.client import LocalCacheInfo, RosFileServerLoader
from compas_fab.backends import RosClient
from compas_robots import RobotModel
from compas_robots.files import URDF

with RosClient() as client:

    robot = client.load_robot()
    robot.info()

    print(robot.name)

    local_cache_directory = None
    local_cache_directory = os.path.join(os.path.expanduser("~"), "robot_package")
    print("Local cache directory:", local_cache_directory)

    loader = RosFileServerLoader(client, True, local_cache_directory)
    # The loader will retrieve URDF and SRDF from the ros server
    urdf_string = loader.load_urdf("/robot_description")
    srdf_string = loader.load_srdf("/robot_description_semantic")
    # The meshes will be loaded over ros to the local cache directory
    robot.model.load_geometry(loader)

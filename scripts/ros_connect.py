# This script extracts the URDF from a MoveIt! instance running in docker

from compas_fab.backends.ros.client import LocalCacheInfo, RosFileServerLoader
from compas_fab.backends import RosClient

with RosClient() as client:

    robot = client.load_robot()
    robot.info()

    print (robot.name)

    local_cache_directory = None
    urdf_param_name="/robot_description",
    srdf_param_name="/robot_description_semantic",

    cache_info = LocalCacheInfo.from_local_cache_directory(local_cache_directory)
    use_local_cache = cache_info.use_local_cache
    robot_name = cache_info.robot_name
    local_cache_directory = cache_info.local_cache_directory

    loader = RosFileServerLoader(client, True, local_cache_directory)
    loader.robot_name = robot_name
    urdf = loader.load_urdf(urdf_param_name)
    srdf = loader.load_srdf(srdf_param_name)
#  ----------------


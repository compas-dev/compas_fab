# This script extracts the URDF from a MoveIt! instance running in docker

# The purpose of this script is to extract the URDF and SRDF from a running MoveIt! instance.
# Many of the industrial robots have a MoveIt! configuration that can be found
# in the ROS package of the robot. However, there are some cases where XARCO files
# are used to define the robot, and the URDF and SRDF are not available.

# This script uses the compas_fab library to extract the URDF and SRDF from a running
# MoveIt! instance. In addition, the meshes are also downloaded and saved in a local
# directory. The extracted files can then be used to create a RobotModel instance
# without the need of a MoveIt! instance nor a ROS installation. This is useful for
# applications where the robot model is used with the PyBullet backend, or when the
# robot model is used for visualization (e.g. in Rhino), or when performing forward
# kinematics calculations that do not require any backends.

# The out name and path of the URDF and SRDF files are governed by the RosFileServerLoader
# class.
# The extract *package* root folder is "\robot_packages\{robot_name}" where {robot_name} is the name of the robot.
# The extracted URDF is located in "\robot_packages\{robot_name}\urdf\robot_description.urdf"
# The extracted SRDF is located in "\robot_packages\{robot_name}\robot_description_semantic.srdf"
# The extracted meshes are stored in paths relative to the package root and are defined in the URDF file.

import os

from compas_fab.backends import RosClient
from compas_fab.backends.ros.client import RosFileServerLoader

HERE = os.path.dirname(__file__)


with RosClient() as client:
    # Standard way to load a robot from a MoveIt! instance.
    robot_cell = client.load_robot_cell()
    robot_model = robot_cell.robot_model
    robot_cell.print_info()
    print(robot_model.name)

    # The RosFileServerLoader is used with a modified local_cache_directory argument
    local_cache_directory = os.path.join(HERE, "robot_packages")
    print("Saved to directory:", os.path.join(local_cache_directory, robot_model.name))
    loader = RosFileServerLoader(client, True, local_cache_directory)

    # The loader will retrieve URDF and SRDF from the ros server
    urdf_string = loader.load_urdf("/robot_description")
    srdf_string = loader.load_srdf("/robot_description_semantic")

    # The meshes will be loaded from ROS to the local cache directory
    mesh_precision = 12
    robot_model.load_geometry(loader, precision=mesh_precision)

    robot_model.to_urdf_file(loader._urdf_filename, prettify=True)

    print("Extraction Complete.")

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

import itertools
import os

from compas.datastructures import Mesh

from compas_fab.backends.ros.client import RosFileServerLoader
from compas_fab.backends import RosClient

HERE = os.path.dirname(__file__)


with RosClient() as client:
    # Standard way to load a robot from a MoveIt! instance.
    robot_cell = client.load_robot_cell()
    robot_cell.print_info()
    print(robot_cell.robot_model.name)

    # The RosFileServerLoader is used with a modified local_cache_directory argument
    local_cache_directory = os.path.join(HERE, "robot_packages")
    print("Saved to directory:", os.path.join(local_cache_directory, robot.name))
    loader = RosFileServerLoader(client, True, local_cache_directory)

    # The loader will retrieve URDF and SRDF from the ros server
    urdf_string = loader.load_urdf("/robot_description")
    srdf_string = loader.load_srdf("/robot_description_semantic")

    # The meshes will be loaded from ROS to the local cache directory
    mesh_precision = 12
    robot.model.load_geometry(loader, precision=mesh_precision)

    # Convert DAE meshes files to OBJ and update the URDF file
    # The new OBJ files will be stored next to the original DAE files appended with ".obj" extension
    for link in robot.model.links:
        for element in itertools.chain(link.collision, link.visual):
            shape = element.geometry.shape
            if "filename" in dir(shape):
                # Convert dae to stl
                if shape.filename.endswith(".dae"):
                    filename_in_package = shape.filename.split("package://")[1]
                    local_path_to_stl = os.path.join(local_cache_directory, robot.name, filename_in_package + ".obj")
                    # Check if the STL file already exists (e.g. RFL have multiple links with the same mesh file)
                    if not os.path.exists(local_path_to_stl):
                        meshes = loader.load_meshes(shape.filename, precision=mesh_precision)
                        # Join all meshes into one for OBJ conversion (Note: Do not weld the vertices)
                        combined_mesh = Mesh()
                        for mesh in meshes:
                            combined_mesh.join(mesh, precision=mesh_precision)
                        combined_mesh.to_obj(local_path_to_stl, precision=mesh_precision)
                        print("> DAE Converted to OBJ file and stored in : ", local_path_to_stl)
                    # Change the filename in the URDF
                    shape.filename = shape.filename + ".obj"

    robot.model.to_urdf_file(loader._urdf_filename, prettify=True)

    print("Extraction Complete.")

import compas
from compas.robots import RobotModel
from compas_fab.backends import RosClient
from compas_fab.backends import RosFileServerLoader

# Set high precision to import meshes defined in meters
compas.PRECISION = '12f'

with RosClient() as ros:
    # Load URDF from ROS
    loader = RosFileServerLoader(ros)
    urdf = loader.load_urdf()

    # Create robot model from URDF and load geometry
    model = RobotModel.from_urdf_string(urdf)
    model.load_geometry(loader)

    print(model)

import compas
from compas.robots import RobotModel
from compas_fab.backends import RosClient
from compas_fab.robots import RosFileServerLoader
from compas_fab.robots import RobotSemantics

# Set high precision to import meshes defined in meters
compas.PRECISION = '12f'

with RosClient() as ros:
    # Configure loader with Ros File Server
    loader = RosFileServerLoader(ros)

    urdf = loader.load_urdf()
    srdf = loader.load_srdf()

    # Create robot model from URDF & SRDF
    model = RobotModel.from_urdf_string(urdf)
    semantics = RobotSemantics.from_srdf_string(srdf, model)

    # Also load geometry
    model.load_geometry(loader)

    print(model)
    print(semantics)

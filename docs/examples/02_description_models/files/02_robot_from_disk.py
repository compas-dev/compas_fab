from compas_robots import RobotModel
from compas_robots.resources import LocalPackageMeshLoader

import compas_fab

# Locate the URDF file inside compas fab installation
urdf = compas_fab.get("universal_robot/ur_description/urdf/ur10e.urdf")

# Create robot model from URDF
model = RobotModel.from_urdf_file(urdf)

# Also load geometry
loader = LocalPackageMeshLoader(compas_fab.get("universal_robot"), "ur_description")
model.load_geometry(loader, precision=12)

print(model)

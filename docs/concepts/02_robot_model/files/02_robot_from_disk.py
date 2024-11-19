from compas_robots import RobotModel
from compas_robots.resources import LocalPackageMeshLoader

import compas_fab

# Locate the URDF file inside compas fab installation
urdf = compas_fab.get("robot_library/ur10e_robot/urdf/robot_description.urdf")

# Create robot model from URDF
model = RobotModel.from_urdf_file(urdf)

# Also load geometry
support_package_name = ""
loader = LocalPackageMeshLoader(compas_fab.get("robot_library/ur10e_robot"), support_package_name)
model.load_geometry(loader, precision=12)

print(model)

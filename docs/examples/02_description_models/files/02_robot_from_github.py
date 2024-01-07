from compas_robots.resources import GithubPackageMeshLoader
from compas_robots import RobotModel

# Select Github repository, package and branch where the model is stored
repository = "ros-industrial/abb"
package = "abb_irb6600_support"
branch = "kinetic-devel"

github = GithubPackageMeshLoader(repository, package, branch)
urdf = github.load_urdf("irb6640.urdf")

# Create robot model from URDF
model = RobotModel.from_urdf_file(urdf)

# Also load geometry
model.load_geometry(github, precision=12)

print(model)

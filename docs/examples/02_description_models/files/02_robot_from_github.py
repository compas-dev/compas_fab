import compas
from compas.robots import GithubPackageMeshLoader
from compas.robots import RobotModel

# Set high precision to import meshes defined in meters
compas.PRECISION = '12f'

# Select Github repository, package and branch where the model is stored
repository = 'ros-industrial/abb'
package = 'abb_irb6600_support'
branch = 'kinetic-devel'

github = GithubPackageMeshLoader(repository, package, branch)
urdf = github.load_urdf('irb6640.urdf')

# Create robot model from URDF
model = RobotModel.from_urdf_file(urdf)

# Also load geometry
model.load_geometry(github)

print(model)

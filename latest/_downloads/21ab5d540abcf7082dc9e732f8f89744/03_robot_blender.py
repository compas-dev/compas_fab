from compas_robots import RobotModel
from compas_robots.resources import GithubPackageMeshLoader
from compas_robots.blender import RobotModelArtist

r = "ros-industrial/abb"
p = "abb_irb6600_support"
b = "kinetic-devel"

github = GithubPackageMeshLoader(r, p, b)
urdf = github.load_urdf("irb6640.urdf")

model = RobotModel.from_urdf_file(urdf)
model.load_geometry(github, precision=12)

RobotModelArtist(model).draw_visual()

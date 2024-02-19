from compas_robots import RobotModel
from compas_robots.resources import GithubPackageMeshLoader
from compas_rhino.artists import RobotModelArtist

r = "ros-industrial/abb"
p = "abb_irb6600_support"
b = "kinetic-devel"

github = GithubPackageMeshLoader(r, p, b)
urdf = github.load_urdf("irb6640.urdf")

robot = RobotModel.from_urdf_file(urdf)
robot.load_geometry(github, precision=12)

RobotModelArtist(robot).draw_visual()

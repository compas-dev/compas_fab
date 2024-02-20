from compas.scene import Scene
from compas_robots import RobotModel
from compas_robots.resources import GithubPackageMeshLoader

r = "ros-industrial/abb"
p = "abb_irb6600_support"
b = "kinetic-devel"

github = GithubPackageMeshLoader(r, p, b)
urdf = github.load_urdf("irb6640.urdf")

model = RobotModel.from_urdf_file(urdf)
model.load_geometry(github, precision=12)

scene = Scene()
scene_object = scene.add(model)
scene.draw()
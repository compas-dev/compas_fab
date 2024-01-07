import compas
compas.PRECISION = '12f'

from compas_robots import *
from compas_rhino.artists import RobotModelArtist

r = 'ros-industrial/abb'
p = 'abb_irb6600_support'
b = 'kinetic-devel'

github = GithubPackageMeshLoader(r, p, b)
urdf = github.load_urdf('irb6640.urdf')

robot = RobotModel.from_urdf_file(urdf)
robot.load_geometry(github)

RobotModelArtist(robot).draw_visual()

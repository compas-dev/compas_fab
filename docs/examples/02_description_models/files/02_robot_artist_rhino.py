import compas
compas.PRECISION = '12f'

from compas.robots import *
from compas_fab.rhino import RobotArtist

r = 'ros-industrial/abb'
p = 'abb_irb6600_support'
b = 'kinetic-devel'

github = GithubPackageMeshLoader(r, p, b)
urdf = github.load_urdf('irb6640.urdf')

robot = RobotModel.from_urdf_file(urdf)
robot.load_geometry(github)

RobotArtist(robot).draw_visual()

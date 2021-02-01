import compas
compas.PRECISION = '12f'

from compas.robots import *
from compas_blender.artists import RobotModelArtist

r = 'ros-industrial/abb'
p = 'abb_irb6600_support'
b = 'kinetic-devel'

github = GithubPackageMeshLoader(r, p, b)
urdf = github.load_urdf('irb6640.urdf')

model = RobotModel.from_urdf_file(urdf)
model.load_geometry(github)

RobotModelArtist(model).draw_visual()

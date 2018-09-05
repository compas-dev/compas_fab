from compas_ghpython import mesh_draw
from compas_ghpython.geometry import xform_from_transformation

from compas_fab.robots.artists import BaseRobotArtist


class RobotArtist(BaseRobotArtist):
    def __init__(self, robot):
        super(RobotArtist, self).__init__(robot)

    def transform(self, native_mesh, transformation):
        T = xform_from_transformation(transformation)
        native_mesh.Transform(T)

    def draw_mesh(self, compas_mesh):
        return mesh_draw(compas_mesh)

    # def set_color(self, color_rgba):
    #     r, g, b, a = color_rgba

from compas.geometry import Frame
from compas.geometry.xforms import Transformation

from compas.datastructures import mesh_transform
from compas.datastructures import mesh_transformed

"""
TODO:
* How to deal with more tcps for one tool?
* What about a tool with rotating/moving parts?
* Add forward and inverse kinematic?
* load from urdf
"""
__all__ = ['Tool']

class Tool(object):
    """Represents the base class for robot end-effectors.

    Attributes:
        tcp_frame (:class:`Frame`): The tcp_frame defines the tool coordinate
            system with the tool center point (tcp) as origin and is relative
            to the tool0_frame, the mounting frame on the robot's flange.
        meshes (:obj:`list` of :class:`Mesh`, optional): The geometry of the
            tool positioned such that the tool0_frame (flange) is in worldXY
            origin.
        draw_function (function, optional): The function to draw the meshes in
            the respective CAD environment. Defaults to None.
        transform_function (function, optional): The function to transform
            the meshes of the respective CAD environment. Defaults to None.

    Examples:
        >>> from compas_ghpython.helpers import mesh_draw
        >>> from compas_ghpython.helpers import mesh_transform
        >>> frame = Frame([1, 1, 1], [0.68, 0.68, 0.27], [-0.67, 0.73, -0.15])
        >>> meshes = [Mesh.from_obj(obj_file)]
        >>> tool = Tool(frame, meshes, mesh_draw, mesh_transform)
    """

    def __init__(self, tcp_frame, meshes=[],
            draw_function=None, transform_function=None):

        self.tcp_frame = tcp_frame
        self.tool0_frame = Frame.worldXY()
        self.meshes = meshes

        # Either both draw and transform functions have to be set or none
        if draw_function and not transform_function:
            raise Exception("The transform function has to be set as well.")
        if not draw_function and transform_function:
            raise Exception("The draw function has to be set as well.")
        self.draw_function = draw_function
        self.transform_function = transform_function

    @property
    def transformation_tool0_tcp(self):
        return Transformation.from_frame_to_frame(self.tcp_frame, self.tool0_frame)

    @property
    def transformation_tcp_tool0(self):
        return Transformation.from_frame_to_frame(self.tool0_frame, self.tcp_frame)

    def load_model(self):
        """Load the geometry (meshes) of the tool.
        """
        raise NotImplementedError

    def get_transformed_meshes(self, transformation):
        """Returns the transformed meshes based on the passed transformation.

        Args:
            transformation (:class:`Transformation`): The transformation to
                reach the robot's tool0_frame.

        Returns:
            meshes (:obj:`list` of :class:`Mesh`): The list of meshes in the
                respective class of the CAD environment
        """
        transformed_meshes = []
        if self.transform_function:
            for mesh in self.meshes:
                transformed_mesh = self.transform_function(mesh, transformation, copy=True)
                transformed_meshes.append(transformed_mesh)
        else:
            for mesh in self.meshes:
                transformed_mesh = mesh_transform(mesh, transformation, copy=True)
                transformed_meshes.append(transformed_mesh)
        return transformed_meshes

if __name__ == "__main__":

    pass

from compas_fab.fab.geometry import Frame, Transformation
from compas_fab.fab.geometry.helpers import mesh_update_vertices
from compas_fab.fab.geometry.transformation import transform_xyz


class Tool(object):
    """
    This is the base class for tools / robot's end-effectors.
    It consists of:
    - geometry (meshes)
    - frame
    - transformation matrix

    TODO:
        - more tcps for one tool!
        - what about a tool with rotating/moving parts?
        - add tool kinematic?
    """

    def __init__(self, tcp_frame):

        self.model = []
        self.tool0_frame = Frame.worldXY()
        self.tcp_frame = tcp_frame
        self.transformation_tool0_tcp = Transformation.from_frame_to_frame(self.tcp_frame, self.tool0_frame)
        self.transformation_tcp_tool0 = Transformation.from_frame_to_frame(self.tool0_frame, self.tcp_frame)

    def load_model(self, xdraw_function=None):
        """Load the geometry (meshes) of the tool.

        Args:
            xdraw_function (function, optional): The function to draw the
                meshes in the respective CAD environment. Defaults to None.
        """
        raise NotImplementedError

    def get_transformed_model(self, transformation, xtransform_function=None):
        """Get the transformed meshes of the tool model.

        Args:
            transformation (:class:`Transformation`): The transformation to
                reach tool0_frame.
            xform_function (function name, optional): the name of the function
                used to transform the model. Defaults to None.

        Returns:
            model (:obj:`list` of :class:`Mesh`): The list of meshes in the
                respective class of the CAD environment
        """
        tmodel = []
        if xtransform_function:
            for m in self.model:
                tmodel.append(xtransform_function(m, transformation, copy=True))
        else:
            for m in self.model:
                mtxyz = transform_xyz(m.xyz, transformation)
                faces = [m.face_vertices(fkey) for fkey in m.faces()]
                tmodel.append(Mesh.from_vertices_and_faces(mtxyz, faces))
        return tmodel

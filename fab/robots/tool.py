from compas_fab.fab.geometry import Frame, Transformation
from compas_fab.fab.geometry.helpers import mesh_update_vertices


class Tool(object):
    """
    This is the base class for tools / robot's end-effectors.
    It consists of:
    - geometry (meshes)
    - frame
    - transformation matrix

    TODO:
        - more tcps for tool!
        - what about a tool with rotating/moving parts?
        - add tool kinematic?
    """

    def __init__(self, tcp_frame):

        self.model = None
        self.tool0_frame = Frame.worldXY()
        self.tcp_frame = tcp_frame
        # TODO: Gonzalo: I find the single-letter part of the name a bit confusing. Maybe rename this to
        # self.transformation_to_tool0 and self.transformation_from_tool0 (or similar)?
        self.transformation_T0_T = Transformation.from_frame_to_frame(self.tcp_frame, self.tool0_frame)
        self.transformation_T_T0 = Transformation.from_frame_to_frame(self.tool0_frame, self.tcp_frame)

    def load_model(self):
        pass

    @property
    def has_model(self):
        return bool(self.model)

    def get_transformed_tool_model(self, tcp_frame):
        if self.has_model:
            T = Transformation.from_frame(tcp_frame) * self.transformation_T_T0
            model_xyz = T.transform(self.model_xyz)
            mesh_update_vertices(self.model, model_xyz)
            return self.model
        else:
            return None
            
        
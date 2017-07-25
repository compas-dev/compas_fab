from compas_fabrication.fabrication.robots import Tool
from compas_fabrication.fabrication.geometry import Frame
from compas.datastructures.mesh import Mesh
import os


class MeasurementTool(Tool):
    """A measurement tool for the UR robot.
    """
    def __init__(self):
        origin = [0.0, 0.0, 115.6]
        xaxis = [0.0, 1., 0.0]
        yaxis = [1., 0.0, 0.0]
        super(MeasurementTool, self).__init__(Frame(origin, xaxis, yaxis))
        self.load_model()
    
    def load_model(self):
        self.model_loaded = True
        path = os.path.join(os.path.dirname(__file__), "models", "measurement_tool.obj")
        self.model = Mesh.from_obj(path)
        self.model_xyz = self.model.xyz
        
"""
measurement_tool = Tool(Frame(origin, xaxis, yaxis))
path = os.path.join(os.path.dirname(__file__), "models", "measurement_tool.obj")
measurement_tool.meshes = Mesh.from_obj(path)
"""

if __name__ == "__main__":
    tool = MeasurementTool()
    print tool
    print tool.tcp_frame
    tool.get_transformed_tool_model(tool.tcp_frame)
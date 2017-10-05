from __future__ import print_function
import os
from compas_fabrication.fabrication.robots import Tool
from compas_fabrication.fabrication.geometry import Frame
from compas.datastructures.mesh import Mesh
from compas_fabrication import get_data

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
        datapath = get_data("robots/ur/tools/measurement_tool.obj")
        self.model = Mesh.from_obj(datapath)
        self.model_xyz = self.model.xyz
        
if __name__ == "__main__":
    tool = MeasurementTool()
    tool.get_transformed_tool_model(tool.tcp_frame)
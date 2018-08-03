from __future__ import print_function
from compas_fab.fab.robots import Tool
from compas_fab.fab.geometry import Frame
from compas.datastructures.mesh import Mesh
from compas_fab import get_data


class MeasurementTool(Tool):
    """A measurement tool for the UR robot.
    """
    def __init__(self):
        origin = [0.0, 0.0, 115.6]
        xaxis = [0.0, 1., 0.0]
        yaxis = [1., 0.0, 0.0]
        super(MeasurementTool, self).__init__(Frame(origin, xaxis, yaxis))

    def load_model(self, xdraw_function=None):
        """Load the geometry (meshes) of the tool.

        Args:
            xdraw_function (function, optional): The function to draw the
                meshes in the respective CAD environment. Defaults to None.
        """
        datapath = get_data("robots/ur/tools/measurement_tool.obj")
        self.model = [Mesh.from_obj(datapath)]

        # draw the geometry in the respective CAD environment
        if xdraw_function:
            for i, m in enumerate(self.model):
                self.model[i] = xdraw_function(m)

if __name__ == "__main__":
    tool = MeasurementTool()
    tool.get_transformed_tool_model(tool.tcp_frame)

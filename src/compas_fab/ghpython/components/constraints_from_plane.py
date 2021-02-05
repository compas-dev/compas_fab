import math

import Grasshopper
import System
from ghpythonlib.componentbase import dotnetcompiledcomponent as component

from compas.geometry import Frame
from compas_fab.ghpython.components.icons import contraints_from_plane_icon


class ConstraintsFromPlane(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Constraints From Plane",
                                                       "Constraints From Plane",
                                                       """Create a position and an orientation constraint from a plane calculated for the group's end-effector link.""",
                                                       "COMPAS FAB",
                                                       "Planning")

    def get_ComponentGuid(self):
        return System.Guid("0e4bd23a-a653-42b9-adf6-d850babc41e6")

    def SetUpParam(self, p, name, nickname, description):
        p.Name = name
        p.NickName = nickname
        p.Description = description
        p.Optional = True

    def RegisterInputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "robot", "robot", "The robot.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Plane()
        self.SetUpParam(p, "plane", "plane", "The plane from which we create position and orientation constraints.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "group", "group", "The planning group for which we specify the constraint. Defaults to the robot's main planning group.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Number()
        self.SetUpParam(p, "tolerance_position", "tolerance_position", "The allowed tolerance to the frame's position (defined in the robot's units). Defaults to 0.001")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Number()
        self.SetUpParam(p, "tolerance_xaxis", "tolerance_xaxis", "Error tolerance of the frame's x-axis in degrees.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Number()
        self.SetUpParam(p, "tolerance_yaxis", "tolerance_yaxis", "Error tolerance of the frame's y-axis in degrees.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Number()
        self.SetUpParam(p, "tolerance_zaxis", "tolerance_zaxis", "Error tolerance of the frame's z-axis in degrees.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "constraints", "constraints", "A list containing a position and an orientation constraint.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        p3 = self.marshal.GetInput(DA, 3)
        p4 = self.marshal.GetInput(DA, 4)
        p5 = self.marshal.GetInput(DA, 5)
        p6 = self.marshal.GetInput(DA, 6)
        result = self.RunScript(p0, p1, p2, p3, p4, p5, p6)

        if result is not None:
            self.marshal.SetOutput(result, DA, 0, True)

    def get_Internal_Icon_24x24(self):
        return contraints_from_plane_icon

    def RunScript(self, robot, plane, group, tolerance_position, tolerance_xaxis, tolerance_yaxis, tolerance_zaxis):
        goal_constraints = None
        if robot and plane:
            tolerance_position = tolerance_position or 0.001
            tolerance_xaxis = tolerance_xaxis or 1.
            tolerance_yaxis = tolerance_yaxis or 1.
            tolerance_zaxis = tolerance_zaxis or 1.

            frame = Frame(plane.Origin, plane.XAxis, plane.YAxis)
            tolerances_axes = [math.radians(tolerance_xaxis), math.radians(tolerance_yaxis), math.radians(tolerance_zaxis)]
            goal_constraints = robot.constraints_from_frame(frame, tolerance_position, tolerances_axes, group)

        return goal_constraints

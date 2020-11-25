import Grasshopper
import System
from compas_ghpython import draw_frame
from compas_ghpython import list_to_ghtree
from ghpythonlib.componentbase import dotnetcompiledcomponent as component

from compas_fab.ghpython.components.icons import default_icon


class TrajectoryVisualize(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Trajectory Visualize",
                                                       "Trajectory Visualize",
                                                       """Visualizes the trajectory.""",
                                                       "COMPAS FAB",
                                                       "Display")

    def get_ComponentGuid(self):
        return System.Guid("ecc3982a-7de6-4de0-80fd-b4505383b85d")

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

        p = Grasshopper.Kernel.Parameters.Param_String()
        self.SetUpParam(p, "group", "group", "The planning group for which this trajectory was planned.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "trajectory", "trajectory", "The calculated trajectory.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):
        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "start_configuration", "start_configuration", "The start configuration of the trajectory.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "configurations", "configurations", "The full configurations along the trajectory.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Number()
        self.SetUpParam(p, "fraction", "fraction", "Indicates the percentage of requested trajectory that was calculated, e.g. 1 means the full trajectory was found.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Number()
        self.SetUpParam(p, "time", "time", "The time which is needed to execute that trajectory at full speed.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Plane()
        self.SetUpParam(p, "planes", "planes", "The planes of the robot's end-effector.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "P", "P", "The positions along the trajectory")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "V", "V", "The velocities along the trajectory")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "A", "A", "The accellerations along the trajectory")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        result = self.RunScript(p0, p1, p2)

        if result is not None:
            if not hasattr(result, '__getitem__'):
                self.marshal.SetOutput(result, DA, 0, True)
            else:
                self.marshal.SetOutput(result[0], DA, 0, True)
                self.marshal.SetOutput(result[1], DA, 1, True)
                self.marshal.SetOutput(result[2], DA, 2, True)
                self.marshal.SetOutput(result[3], DA, 3, True)
                self.marshal.SetOutput(result[4], DA, 4, True)
                self.marshal.SetOutput(result[5], DA, 5, True)
                self.marshal.SetOutput(result[6], DA, 6, True)
                self.marshal.SetOutput(result[6], DA, 7, True)

    def get_Internal_Icon_24x24(self):
        return default_icon

    def RunScript(self, robot, group, trajectory):
        start_configuration = None
        configurations = []
        fraction = 0.
        time = 0.

        planes = []
        positions = []
        velocities = []
        accelerations = []

        if robot and trajectory:
            group = group or robot.main_group_name

            for c in trajectory.points:
                configurations.append(robot.merge_group_with_full_configuration(c, trajectory.start_configuration, group))
                frame = robot.forward_kinematics(c, group, options=dict(solver='model'))
                planes.append(draw_frame(frame))
                positions.append(c.positions)
                velocities.append(c.velocities)
                accelerations.append(c.accelerations)

            start_configuration = trajectory.start_configuration
            fraction = trajectory.fraction
            time = trajectory.time_from_start

        P = list_to_ghtree(list(zip(*positions)))
        V = list_to_ghtree(list(zip(*velocities)))
        A = list_to_ghtree(list(zip(*accelerations)))

        # return outputs if you have them; here I try it for you:
        return (start_configuration, configurations, fraction, time, planes, P, V, A)

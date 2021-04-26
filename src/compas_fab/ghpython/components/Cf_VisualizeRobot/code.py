import Grasshopper
import System
from ghpythonlib.componentbase import dotnetcompiledcomponent as component

from compas.geometry import Frame
from compas.geometry import Transformation
from compas_ghpython.artists import MeshArtist
from compas_fab.ghpython.components.icons import robot_visualize_icon


class RobotVisualize(component):
    def __new__(cls):
        return Grasshopper.Kernel.GH_Component.__new__(cls,
                                                       "Robot Visualize",
                                                       "Robot Visualize",
                                                       """Visualizes the robot.""",
                                                       "COMPAS FAB",
                                                       "Display")

    def get_ComponentGuid(self):
        return System.Guid("1ba79e0a-3216-4898-a8c3-c6ba069e653d")

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
        self.SetUpParam(p, "group", "group", "The planning group used for end-effector and base visualization.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "configuration", "configuration", "The robot's full configuration.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "attached_collision_meshes", "attached_collision_meshes", "A list of attached collision meshes.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.list
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "show_visual", "show_visual", "Whether or not to show the robot's visual meshes.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "show_collision", "show_collision", "Whether or not to show the robot's collision meshes.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "show_frames", "show_frames", "Whether or not to show the robot's joint frames.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "show_base_frame", "show_base_frame", "Whether or not to show the robot's base frame.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "show_end_effector_frame", "show_end_effector_frame", "Whether or not to show the robot's end-effector frame.")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_Boolean()
        self.SetUpParam(p, "show_acm", "show_acm", "Whether or not to show the attached collision meshes (if any).")
        p.Access = Grasshopper.Kernel.GH_ParamAccess.item
        self.Params.Input.Add(p)

    def RegisterOutputParams(self, pManager):

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "visual", "visual", "The robot's visual meshes.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "collision", "collision", "The robot's collision meshes.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "attached_meshes", "attached_meshes", "The robot's attached meshes.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "frames", "frames", "The robot's joint frames.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "base_frame", "base_frame", "The robot's base frame.")
        self.Params.Output.Add(p)

        p = Grasshopper.Kernel.Parameters.Param_GenericObject()
        self.SetUpParam(p, "ee_frame", "ee_frame", "The robot's end-effector frame.")
        self.Params.Output.Add(p)

    def SolveInstance(self, DA):
        p0 = self.marshal.GetInput(DA, 0)
        p1 = self.marshal.GetInput(DA, 1)
        p2 = self.marshal.GetInput(DA, 2)
        p3 = self.marshal.GetInput(DA, 3)
        p4 = self.marshal.GetInput(DA, 4)
        p5 = self.marshal.GetInput(DA, 5)
        p6 = self.marshal.GetInput(DA, 6)
        p7 = self.marshal.GetInput(DA, 7)
        p8 = self.marshal.GetInput(DA, 8)
        p9 = self.marshal.GetInput(DA, 9)
        result = self.RunScript(p0, p1, p2, p3, p4, p5, p6, p7, p8, p9)

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

    def get_Internal_Icon_24x24(self):
        return robot_visualize_icon

    def RunScript(self, robot, group, configuration, attached_collision_meshes, show_visual, show_collision,
                  show_frames, show_base_frame, show_end_effector_frame, show_acm):

        visual = None
        collision = None
        attached_meshes = None
        frames = None
        base_frame = None
        ee_frame = None

        if robot:
            show_visual = show_visual or True
            configuration = configuration or robot.zero_configuration()

            robot.update(configuration, visual=show_visual, collision=show_collision)
            frames = robot.transformed_frames(configuration, group)

            if show_visual:
                visual = robot.artist.draw_visual()

            if show_collision:
                collision = robot.artist.draw_collision()

            if show_base_frame:
                base_frame = frames[0]

            if show_end_effector_frame:
                ee_frame = robot.forward_kinematics(configuration, group, options=dict(solver='model'))

            if not show_frames:
                frames = None

            if show_acm:
                attached_meshes = []
                for acm in attached_collision_meshes:
                    frame = robot.forward_kinematics(configuration, options=dict(solver='model', link_name=acm.link_name))
                    T = Transformation.from_frame_to_frame(Frame.worldXY(), frame)
                    mesh = acm.collision_mesh.mesh.transformed(T)
                    attached_meshes.append(MeshArtist(mesh).draw())

        return (visual, collision, attached_meshes, frames, base_frame, ee_frame)

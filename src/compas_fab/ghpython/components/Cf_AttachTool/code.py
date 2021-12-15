"""
Attach a tool to the robot.

COMPAS FAB v0.21.1
"""
from ghpythonlib.componentbase import executingcomponent as component
from compas_rhino.conversions import RhinoMesh
from compas_rhino.conversions import plane_to_compas_frame

from compas.geometry import Frame
from compas_fab.robots import PlanningScene
from compas_fab.robots import Tool


class AttachToolComponent(component):
    def RunScript(self, robot, visual_mesh, collision_mesh, tcf_plane):
        if robot and robot.client and robot.client.is_connected and visual_mesh:
            if not collision_mesh:
                collision_mesh = visual_mesh

            c_visual_mesh = RhinoMesh.from_geometry(visual_mesh).to_compas()
            c_collision_mesh = RhinoMesh.from_geometry(collision_mesh).to_compas()

            if not tcf_plane:
                frame = Frame.worldXY()
            else:
                frame = plane_to_compas_frame(tcf_plane)
            tool = Tool(c_visual_mesh, frame, c_collision_mesh)

            scene = PlanningScene(robot)
            robot.attach_tool(tool)
            scene.add_attached_tool()

        return robot

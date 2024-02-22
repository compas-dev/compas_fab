"""
Attach a tool to the robot.

COMPAS FAB v1.0.2
"""

from compas.geometry import Frame
from compas_rhino.conversions import mesh_to_compas
from compas_rhino.conversions import plane_to_compas_frame
from ghpythonlib.componentbase import executingcomponent as component

from compas_fab.robots import Tool


class AttachToolComponent(component):
    def RunScript(self, robot, visual_mesh, collision_mesh, tcf_plane, group, connected_to):
        if robot and robot.client and robot.client.is_connected and visual_mesh:
            if not collision_mesh:
                collision_mesh = visual_mesh

            c_visual_mesh = mesh_to_compas(visual_mesh)
            c_collision_mesh = mesh_to_compas(collision_mesh)

            if not tcf_plane:
                frame = Frame.worldXY()
            else:
                frame = plane_to_compas_frame(tcf_plane)
            tool = Tool(c_visual_mesh, frame, c_collision_mesh, connected_to=connected_to)

            robot.attach_tool(tool, group)

        return robot

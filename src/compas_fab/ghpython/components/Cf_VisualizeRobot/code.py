"""
Visualizes the robot.

COMPAS FAB v0.18.0
"""
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_ghpython.artists import MeshArtist
from ghpythonlib.componentbase import executingcomponent as component


class RobotVisualize(component):
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

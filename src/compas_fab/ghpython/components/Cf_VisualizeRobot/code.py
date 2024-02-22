"""
Visualizes the robot.

COMPAS FAB v1.0.2
"""

import time

from compas.geometry import Frame
from compas.geometry import Transformation
from compas.scene import SceneObject
from compas_ghpython import create_id
from compas_rhino.conversions import frame_to_rhino_plane
from ghpythonlib.componentbase import executingcomponent as component
from scriptcontext import sticky as st

from compas_fab.backends import BackendFeatureNotSupportedError
from compas_fab.robots import PlanningScene


class RobotVisualize(component):
    def RunScript(
        self,
        robot,
        group,
        configuration,
        show_visual,
        show_collision,
        show_frames,
        show_base_frame,
        show_end_effector_frame,
        show_cm,
        show_acm,
    ):
        visual = None
        collision = None
        collision_meshes = None
        attached_meshes = None
        frames = None
        base_frame = None
        ee_frame = None

        if robot:
            show_visual = True if show_visual is None else show_visual
            show_cm = True if show_cm is None else show_cm
            show_acm = True if show_acm is None else show_acm
            configuration = configuration or robot.zero_configuration()

            robot.update(configuration, visual=show_visual, collision=show_collision)
            compas_frames = robot.transformed_frames(configuration, group)

            if show_visual:
                visual = robot.scene_object.draw_visual()

            if show_collision:
                collision = robot.scene_object.draw_collision()

            if show_base_frame:
                base_compas_frame = compas_frames[0]
                base_frame = frame_to_rhino_plane(base_compas_frame)

            if show_end_effector_frame:
                options = dict(solver="model")
                if robot.attached_tool:
                    options["link"] = robot.attached_tool.connected_to

                ee_compas_frame = robot.forward_kinematics(configuration, group, options=options)
                ee_frame = frame_to_rhino_plane(ee_compas_frame)

            if show_frames:
                frames = []
                for compas_frame in compas_frames[1:]:
                    frame = frame_to_rhino_plane(compas_frame)
                    frames.append(frame)

            cached_scene_key = create_id(self, "cached_scene")

            if show_cm or show_acm:
                cached_scene = st.get(cached_scene_key)
                if not cached_scene:
                    cached_scene = {"time": 0}

                # expire cache if the component has not been executed in the last 2 seconds
                # this allows to slide through a list of configurations
                # without triggering refreshes of the scene in the middle of it
                if time.time() - cached_scene["time"] > 2:
                    update_scene = True
                else:
                    update_scene = False

                if update_scene:
                    scene = PlanningScene(robot)
                    try:
                        scene = robot.client.get_planning_scene()
                    except BackendFeatureNotSupportedError:
                        print(
                            "The selected backend does not support collision meshes. If you need collision mesh support, use a different backend."
                        )
                        scene = None
                        show_cm = False
                        show_acm = False

                if update_scene and show_cm:
                    collision_meshes = []

                    for co in scene.world.collision_objects:
                        header = co.header
                        frame_id = header.frame_id
                        cms = co.to_collision_meshes()

                        for cm in cms:
                            if cm.frame != Frame.worldXY():
                                t = Transformation.from_frame(cm.frame)
                                mesh = cm.mesh.transformed(t)
                            else:
                                mesh = cm.mesh

                            collision_meshes.extend(SceneObject(mesh).draw())

                        cached_scene["cm"] = collision_meshes

                collision_meshes = cached_scene.get("cm", [])

                if update_scene and show_acm:
                    attached_meshes = []

                    for aco in scene.robot_state.attached_collision_objects:
                        for acm in aco.to_attached_collision_meshes():
                            frame_id = aco.object["header"]["frame_id"]
                            frame = robot.forward_kinematics(configuration, options=dict(link=frame_id))
                            t = Transformation.from_frame(frame)

                            # Local CM frame
                            if acm.collision_mesh.frame and acm.collision_mesh.frame != Frame.worldXY():
                                t = t * Transformation.from_frame(acm.collision_mesh.frame)

                            mesh = acm.collision_mesh.mesh.transformed(t)

                            attached_meshes.extend(SceneObject(mesh).draw())

                    cached_scene["acm"] = attached_meshes

                attached_meshes = cached_scene.get("acm", [])

                cached_scene["time"] = time.time()
                st[cached_scene_key] = cached_scene

        return (
            visual,
            collision,
            collision_meshes,
            attached_meshes,
            frames,
            base_frame,
            ee_frame,
        )

"""
Calculate a cartesian motion path (linear in tool space).

COMPAS FAB v1.0.2
"""

from compas_rhino.conversions import plane_to_compas_frame
from ghpythonlib.componentbase import executingcomponent as component
from scriptcontext import sticky as st

from compas_fab.ghpython.components import create_id
from compas_fab.robots import FrameWaypoints


class PlanCartesianMotion(component):
    def RunScript(
        self, robot, planes, start_configuration, group, attached_collision_meshes, path_constraints, max_step, compute
    ):
        key = create_id(self, "trajectory")

        max_step = float(max_step) if max_step else 0.01
        path_constraints = list(path_constraints) if path_constraints else None
        attached_collision_meshes = list(attached_collision_meshes) if attached_collision_meshes else None

        if robot and robot.client and robot.client.is_connected and start_configuration and planes and compute:
            frames = [plane_to_compas_frame(plane) for plane in planes]
            frame_waypoints = FrameWaypoints(frames)
            st[key] = robot.plan_cartesian_motion(
                frame_waypoints,
                start_configuration=start_configuration,
                group=group,
                options=dict(
                    max_step=max_step,
                    path_constraints=path_constraints,
                    attached_collision_meshes=attached_collision_meshes,
                ),
            )

        trajectory = st.get(key, None)
        return trajectory

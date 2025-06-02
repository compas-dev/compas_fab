# r: compas_fab>=1.0.2
"""
Calculate a cartesian motion path (linear in tool space).

COMPAS FAB v1.1.0
"""

import Grasshopper
import System
from compas_ghpython import create_id
from compas_rhino.conversions import plane_to_compas_frame
from scriptcontext import sticky as st


class PlanCartesianMotion(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        robot,
        planes: System.Collections.Generic.List[object],
        start_configuration,
        group: str,
        attached_collision_meshes: System.Collections.Generic.List[object],
        path_constraints: System.Collections.Generic.List[object],
        max_step: float,
        compute: bool,
    ):
        key = create_id(ghenv.Component, "trajectory")  # noqa: F821

        max_step = float(max_step) if max_step else 0.01
        path_constraints = list(path_constraints) if path_constraints else None
        attached_collision_meshes = list(attached_collision_meshes) if attached_collision_meshes else None

        if robot and robot.client and robot.client.is_connected and start_configuration and planes and compute:
            frames = [plane_to_compas_frame(plane) for plane in planes]
            st[key] = robot.plan_cartesian_motion(
                frames,
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

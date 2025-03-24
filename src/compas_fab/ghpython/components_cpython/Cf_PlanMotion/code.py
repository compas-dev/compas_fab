# r: compas_fab>=1.0.2
"""
Calculate a motion path.

COMPAS FAB v1.0.2
"""

import Grasshopper
import System
from scriptcontext import sticky as st

from compas_fab.ghpython.components import create_id


class PlanMotion(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(
        self,
        robot,
        goal_constraints: System.Collections.Generic.List[object],
        start_configuration,
        group: str,
        attached_collision_meshes: System.Collections.Generic.List[object],
        path_constraints: System.Collections.Generic.List[object],
        planner_id: str,
        compute: bool,
    ):
        key = create_id(ghenv.Component, "trajectory")  # noqa: F821

        path_constraints = list(path_constraints) if path_constraints else None
        attached_collision_meshes = list(attached_collision_meshes) if attached_collision_meshes else None
        planner_id = str(planner_id) if planner_id else "RRTConnect"

        if (
            robot
            and robot.client
            and robot.client.is_connected
            and start_configuration
            and goal_constraints
            and compute
        ):
            st[key] = robot.plan_motion(
                goal_constraints,
                start_configuration=start_configuration,
                group=group,
                options=dict(
                    attached_collision_meshes=attached_collision_meshes,
                    path_constraints=path_constraints,
                    planner_id=planner_id,
                ),
            )

        trajectory = st.get(key, None)
        return trajectory

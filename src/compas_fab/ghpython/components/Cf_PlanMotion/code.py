"""
Calculate a motion path.

COMPAS FAB v0.19.0
"""
from ghpythonlib.componentbase import executingcomponent as component
from scriptcontext import sticky as st

from compas_fab.ghpython.components import create_id


class PlanMotion(component):
    def RunScript(self, robot, goal_constraints, start_configuration, group, attached_collision_meshes, path_constraints, planner_id, compute):

        key = create_id(self, 'trajectory')

        path_constraints = list(path_constraints) if path_constraints else None
        attached_collision_meshes = list(attached_collision_meshes) if attached_collision_meshes else None
        planner_id = str(planner_id) if planner_id else 'RRTConnectkConfigDefault'

        if robot and robot.client and robot.client.is_connected and start_configuration and goal_constraints and compute:
            st[key] = robot.plan_motion(goal_constraints,
                                        start_configuration=start_configuration,
                                        group=group,
                                        options=dict(
                                            attached_collision_meshes=attached_collision_meshes,
                                            path_constraints=path_constraints,
                                            planner_id=planner_id,
                                        ))

        trajectory = st.get(key, None)
        return trajectory

"""
Create a planning scene.

COMPAS FAB v0.18.3
"""
from ghpythonlib.componentbase import executingcomponent as component
from scriptcontext import sticky as st

from compas_fab.ghpython.components import create_id
from compas_fab.robots import PlanningScene


class PlanningSceneComponent(component):
    def RunScript(self, robot):
        key = create_id(self, 'planning_scene')
        if robot:
            st[key] = PlanningScene(robot)
        return st.get(key, None)

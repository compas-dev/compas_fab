# r: compas_fab>=1.0.2
"""
Create a planning scene.

COMPAS FAB v1.0.2
"""

import Grasshopper
from compas_ghpython import create_id
from scriptcontext import sticky as st

from compas_fab.robots import PlanningScene


class PlanningSceneComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, robot):
        key = create_id(ghenv.Component, "planning_scene")  # noqa: F821
        if robot:
            st[key] = PlanningScene(robot)
        return st.get(key, None)

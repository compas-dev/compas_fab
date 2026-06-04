# r: compas_fab>=1.1.0
"""
Assemble an ActionChain from a list of Actions and emit the composite trajectory visualisation.

Wire the outputs of Cf_TrajectoryAction / Cf_StateChangeAction components
into `actions` in the order the motion should be executed. The component
threads the cell state through the chain, validates each action, and emits
the assembled chain plus composite outputs for visualisation.

`planes` is a DataTree with one branch per trajectory action (per-action
coloring / filtering work out of the box); `polyline` is a list of
polylines parallel to those branches. `cell_states` is a flat list of
RobotCellState snapshots — one per trajectory point plus one per state
change — so a single index slider + VisualizeRobotCell can scrub through
the whole chain frame by frame.

COMPAS FAB v1.1.0
"""

import Grasshopper
from compas_ghpython import error as gh_error
from ghpythonlib.treehelpers import list_to_tree

from compas_fab.ghpython import trajectory_to_planes_and_polyline
from compas_fab.robots import ActionChain


class ActionChainComponent(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, name: str, start_state, actions, robot_cell, group: str, description: str):
        if not name or start_state is None or not actions:
            return (None, [], None, [], 0.0, start_state)

        try:
            chain = ActionChain(
                name=name.strip(),
                start_state=start_state,
                robot_cell=robot_cell,
                description=(description or "").strip(),
            )
            for action in actions:
                if action is None:
                    continue
                chain.append_action(action)
        except ValueError as e:
            gh_error(ghenv.Component, "Chain assembly failed: {}".format(e))  # noqa: F821
            return (None, [], None, [], 0.0, start_state)

        cell_states = list(chain.iter_cell_states())

        if robot_cell is None:
            return (chain, [], None, cell_states, chain.duration, chain.end_state)

        # Composite visualisation: one branch per trajectory action. Walk the
        # chain so each action's FK uses the correct pre-state.
        planes_per_action = []
        polylines_per_action = []
        running_state = chain.start_state
        for action in chain:
            if not action.is_trajectory:
                running_state = action.post_state
                continue
            planes, polyline = trajectory_to_planes_and_polyline(
                robot_cell, running_state, action.trajectory, group or None
            )
            planes_per_action.append(planes)
            polylines_per_action.append(polyline)
            running_state = action.post_state

        planes_tree = list_to_tree(planes_per_action) if planes_per_action else []
        return (chain, planes_tree, polylines_per_action or None, cell_states, chain.duration, chain.end_state)

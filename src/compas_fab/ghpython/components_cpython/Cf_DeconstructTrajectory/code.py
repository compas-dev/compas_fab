# r: compas_fab>=1.1.0
"""
Expand a JointTrajectory into per-point Configurations and RobotCellStates,
plus the per-point velocity/acceleration/effort profiles.

`configurations` is always populated (one Configuration per trajectory
point). `cell_states` is populated only when `start_state` is wired —
each entry is a copy of `start_state` with `robot_configuration` set to
the corresponding trajectory point, so it can be piped straight into
`VisualizeRobotCell` (e.g. via an index slider to scrub through frames).

`velocities`, `accelerations` and `efforts` are lists of lists — one
inner list per trajectory point, each ordered to match the trajectory's
`joint_names`. They land on the GH canvas as DataTrees (one branch per
point), so a Quick Graph / chart node can plot the profile directly.
Backends that don't populate these fields return lists of zeros.

When the trajectory's joint set is a subset of `start_state.robot_configuration`
(common for robots with extra joints outside the planning group, e.g. Panda's
finger), the values are merged by joint name so the unrelated joints keep
the value they had in `start_state`.

COMPAS FAB v1.1.0
"""

from copy import deepcopy

import Grasshopper
from ghpythonlib.treehelpers import list_to_tree


def _state_with_point(start_state, point, trajectory_joint_names):
    new_state = deepcopy(start_state)
    base = new_state.robot_configuration
    if base is None:
        new_state.robot_configuration = point
        return new_state
    if not point.joint_names and trajectory_joint_names:
        point = deepcopy(point)
        point.joint_names = list(trajectory_joint_names)
    new_state.robot_configuration = base.merged(point)
    return new_state


class DeconstructTrajectory(Grasshopper.Kernel.GH_ScriptInstance):
    def RunScript(self, trajectory, start_state):
        if trajectory is None or not getattr(trajectory, "points", None):
            return ([], [], [], [], [])

        points = trajectory.points
        configurations = list(points)
        velocities = list_to_tree([p.velocities for p in points])
        accelerations = list_to_tree([p.accelerations for p in points])
        efforts = list_to_tree([p.effort for p in points])

        if start_state is None:
            return (configurations, [], velocities, accelerations, efforts)

        trajectory_joint_names = getattr(trajectory, "joint_names", None)
        cell_states = [_state_with_point(start_state, pt, trajectory_joint_names) for pt in points]
        return (configurations, cell_states, velocities, accelerations, efforts)

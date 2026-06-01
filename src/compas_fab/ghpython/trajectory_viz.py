"""Helpers for visualizing planning trajectories on a Grasshopper canvas.

A planned trajectory is just a sequence of `Configuration`s — it tells you
*what joints*, not *where the robot is in space*. To draw the path, each
point has to be passed through forward kinematics with the same cell state
the planner saw (so attached tools / workpieces resolve correctly).

This module wraps that loop in a single helper so the planning components
can offer "planes" and "polyline" outputs without each one re-implementing
the FK walk.
"""

from copy import deepcopy

import compas

if compas.RHINO:
    from Rhino.Geometry import Polyline

from compas_rhino.conversions import frame_to_rhino_plane

from compas_fab.robots import TargetMode


def _state_with_trajectory_point(start_state, point, trajectory_joint_names):
    """Return a copy of ``start_state`` whose ``robot_configuration`` has
    the point's joint values merged in.

    Trajectory points only contain the planning group's joints, while
    the cell state can carry additional configurable joints (Panda's
    finger, dual-arm robots, etc.). Merging by name preserves those.

    ROS `trajectory_msgs/JointTrajectoryPoint` carries no joint_names of
    its own — names live on the parent `JointTrajectory`. Older
    backends therefore yielded points with empty `joint_names`; for
    those we fall back to the parent trajectory's names.
    """
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


def trajectory_to_planes_and_polyline(planner, start_state, trajectory, group=None):
    """Compute Rhino planes and a connecting polyline from a trajectory.

    For each trajectory point, the helper merges the point into a copy
    of ``start_state`` and asks ``planner.forward_kinematics`` for the
    PCF (`target_mode=ROBOT`, the planning group's end-effector link)
    in WCF. The frames are converted to Rhino planes; the polyline is
    built from their origins in order.

    Returns ``(planes, polyline)`` — ``planes`` is a list of
    ``Rhino.Geometry.Plane``, ``polyline`` is a ``Rhino.Geometry.Polyline``
    (or ``None`` when there are fewer than two points). Any failure
    along the way returns ``([], None)`` rather than raising.
    """
    if planner is None or start_state is None or trajectory is None:
        return ([], None)
    if not getattr(trajectory, "points", None):
        return ([], None)

    trajectory_joint_names = getattr(trajectory, "joint_names", None)
    planes = []
    try:
        for point in trajectory.points:
            state = _state_with_trajectory_point(start_state, point, trajectory_joint_names)
            frame = planner.forward_kinematics(
                robot_cell_state=state,
                target_mode=TargetMode.ROBOT,
                group=group or None,
            )
            planes.append(frame_to_rhino_plane(frame))
    except Exception:
        return ([], None)

    polyline = Polyline([plane.Origin for plane in planes]) if len(planes) >= 2 else None
    return (planes, polyline)

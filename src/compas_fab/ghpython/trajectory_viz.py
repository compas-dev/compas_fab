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

from compas.geometry import Polyline
from compas_rhino.conversions import frame_to_rhino_plane
from compas_rhino.conversions import polyline_to_rhino
from compas_robots import Configuration

from compas_fab.robots import TargetMode


def trajectory_to_planes_and_polyline(robot_cell, start_state, trajectory, group=None):
    """Compute Rhino planes and a connecting polyline from a trajectory.

    For each trajectory point, the helper substitutes the point's joint
    values into a single working copy of `start_state` and runs **local**
    forward kinematics via `RobotCell.forward_kinematics_target_frame`
    (`target_mode=ROBOT`) — i.e. the URDF kinematic chain in process, with
    no backend round trip. The frames are converted to Rhino planes; a
    polyline through their origins is computed in `compas` and converted
    to Rhino.

    `start_state` may be `None`; in that case the trajectory's own
    `start_state` (populated by the planner that produced it) is used.

    Returns `(planes, polyline)`. `polyline` is `None` when there are
    fewer than two points. Any failure along the way returns `([], None)`
    rather than raising.
    """
    if robot_cell is None or trajectory is None:
        return ([], None)
    if not getattr(trajectory, "points", None):
        return ([], None)
    if start_state is None:
        start_state = getattr(trajectory, "start_state", None)
    if start_state is None:
        return ([], None)

    # ROS `trajectory_msgs/JointTrajectoryPoint` carries no joint_names —
    # names live on the parent `JointTrajectory`. Fall back accordingly.
    point_names = list(trajectory.points[0].joint_names) or list(getattr(trajectory, "joint_names", None) or [])

    # One deepcopy outside the loop; per-point we only swap the configuration.
    working_state = deepcopy(start_state)
    base = working_state.robot_configuration

    # Pre-compute merge data once. If we can't merge by name, fall back to
    # using the point directly — works when the trajectory and cell share
    # the same joint set.
    merge_data = None
    if base is not None and point_names:
        base_names = list(base.joint_names)
        merge_data = (
            base_names,
            list(base.joint_types),
            list(base.joint_values),
            {name: idx for idx, name in enumerate(base_names)},
        )

    frames = []
    try:
        for point in trajectory.points:
            if merge_data is None:
                working_state.robot_configuration = point
            else:
                base_names, base_types, base_values_template, name_to_pos = merge_data
                values = list(base_values_template)
                for name, value in zip(point_names, point.joint_values):
                    idx = name_to_pos.get(name)
                    if idx is not None:
                        values[idx] = value
                working_state.robot_configuration = Configuration(values, base_types, base_names)
            frames.append(
                robot_cell.forward_kinematics_target_frame(
                    robot_cell_state=working_state,
                    target_mode=TargetMode.ROBOT,
                    group=group or None,
                )
            )
    except Exception:
        return ([], None)

    planes = [frame_to_rhino_plane(f) for f in frames]
    polyline = polyline_to_rhino(Polyline([f.point for f in frames])) if len(frames) >= 2 else None
    return (planes, polyline)

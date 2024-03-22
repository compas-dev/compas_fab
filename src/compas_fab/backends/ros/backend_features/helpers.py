from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import itertools

from compas_robots import Configuration

from compas_fab.backends.ros.exceptions import RosError
from compas_fab.backends.ros.messages import Constraints
from compas_fab.backends.ros.messages import JointConstraint
from compas_fab.backends.ros.messages import MoveItErrorCodes
from compas_fab.backends.ros.messages import OrientationConstraint
from compas_fab.backends.ros.messages import PositionConstraint
from compas_fab.robots import ConfigurationTarget
from compas_fab.robots import ConstraintSetTarget
from compas_fab.robots import Duration
from compas_fab.robots import FrameTarget
from compas_fab.robots import JointTrajectory
from compas_fab.robots import JointTrajectoryPoint
from compas_fab.robots import PointAxisTarget
from compas_fab.robots.constraints import JointConstraint as CF_JointConstraint
from compas_fab.robots.constraints import PositionConstraint as CF_PositionConstraint
from compas_fab.robots.constraints import OrientationConstraint as CF_OrientationConstraint


def validate_response(response):
    """Raise an exception if the response indicates an error condition."""
    if response.error_code != MoveItErrorCodes.SUCCESS:
        raise RosError(response.error_code.human_readable, int(response.error_code))


def convert_target_to_goal_constraints(target, ee_link_name):
    """Convert COMPAS FAB `Target` objects into `Constraint` objects for passing it to MoveIt backend.
    This function is intended to be called only by MoveIt backend when handeling different Target types.

    Parameters
    ----------
    target: :class:`compas_fab.robots.Target`
        The goal for the robot to achieve.

    Returns
    -------
    list of :class:`Constraint`
        Set of Constraint classes
    """

    if isinstance(target, ConstraintSetTarget):
        return target.constraint_set

    elif isinstance(target, ConfigurationTarget):
        configuration = target.target_configuration
        tolerance_above = target.tolerance_above
        tolerance_below = target.tolerance_below
        return CF_JointConstraint.joint_constraints_from_configuration(configuration, tolerance_above, tolerance_below)

    elif isinstance(target, FrameTarget):
        tcf_frame_in_wcf = target.target_frame
        tool_coordinate_frame = target.tool_coordinate_frame
        pc = CF_PositionConstraint.from_frame(
            tcf_frame_in_wcf, target.tolerance_position, ee_link_name, tool_coordinate_frame
        )
        oc = CF_OrientationConstraint.from_frame(
            tcf_frame_in_wcf, [target.tolerance_orientation] * 3, ee_link_name, tool_coordinate_frame
        )
        return [pc, oc]

    elif isinstance(target, PointAxisTarget):
        tcf_point_in_wcf = target.target_point
        tool_coordinate_frame = target.tool_coordinate_frame

        if tool_coordinate_frame:
            raise NotImplementedError(
                "Tool coordinate frame is not yet supported when converting PointAxisTarget to ConstraintSetTarget."
            )

        pc = CF_PositionConstraint.from_point(
            tcf_point_in_wcf, target.tolerance_position, ee_link_name, tool_coordinate_frame
        )
        oc = CF_OrientationConstraint.from_frame(
            tcf_frame_in_wcf, [6.35, 6.35, 0.01], ee_link_name, tool_coordinate_frame
        )
        return [pc, oc]

    else:
        raise NotImplementedError("Target type {} not supported by ROS planning backend.".format(type(target)))


def convert_constraints_to_rosmsg(constraints, header):
    """Convert COMPAS FAB constraints into ROS Messages."""
    if not constraints:
        return None

    ros_constraints = Constraints()
    for c in constraints:
        if c.type == c.JOINT:
            ros_constraints.joint_constraints.append(JointConstraint.from_joint_constraint(c))
        elif c.type == c.POSITION:
            ros_constraints.position_constraints.append(PositionConstraint.from_position_constraint(header, c))
        elif c.type == c.ORIENTATION:
            ros_constraints.orientation_constraints.append(OrientationConstraint.from_orientation_constraint(header, c))
        else:
            raise NotImplementedError

    return ros_constraints


def convert_trajectory_points(points, joint_types):
    result = []

    for pt in points:
        jtp = JointTrajectoryPoint(
            joint_values=pt.positions,
            joint_types=joint_types,
            velocities=pt.velocities,
            accelerations=pt.accelerations,
            effort=pt.effort,
            time_from_start=Duration(pt.time_from_start.secs, pt.time_from_start.nsecs),
        )

        result.append(jtp)

    return result


def convert_trajectory(joints, solution, solution_start_state, fraction, planning_time, source_message):
    trajectory = JointTrajectory()
    trajectory.source_message = source_message
    trajectory.fraction = fraction
    trajectory.joint_names = solution.joint_trajectory.joint_names
    trajectory.planning_time = planning_time

    joint_types = [joints[name] for name in trajectory.joint_names]
    trajectory.points = convert_trajectory_points(solution.joint_trajectory.points, joint_types)

    start_state = solution_start_state.joint_state
    start_state_types = [joints[name] for name in start_state.name]
    trajectory.start_configuration = Configuration(start_state.position, start_state_types, start_state.name)
    trajectory.attached_collision_meshes = list(
        itertools.chain(
            *[aco.to_attached_collision_meshes() for aco in solution_start_state.attached_collision_objects]
        )
    )

    return trajectory

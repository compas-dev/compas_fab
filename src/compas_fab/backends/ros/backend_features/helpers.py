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
from compas_fab.backends.exceptions import MPStartStateInCollisionError
from compas_fab.backends.exceptions import MPTargetInCollisionError
from compas_fab.backends.exceptions import MPSearchTimeOutError
from compas_fab.backends.exceptions import PlanningGroupNotExistsError
from compas_fab.backends.exceptions import MPNoPlanFoundError

DEFAULT_TOLERANCE_ORIENTATION = 0.1
DEFAULT_TOLERANCE_POSITION = 0.01


def validate_response(response):
    """Raise an exception if the response indicates an error condition.
    Errors that are covered by compas_fab error classes are raised as such.
    Otherwise, a generic RosError is raised with the error code.
    """
    if response.error_code != MoveItErrorCodes.SUCCESS:
        if response.error_code == MoveItErrorCodes.PLANNING_FAILED:
            raise MPNoPlanFoundError("MoveItErrorCodes -1 PLANNING_FAILED")
        elif response.error_code == MoveItErrorCodes.TIMED_OUT:
            raise MPSearchTimeOutError("MoveItErrorCodes -6 TIMED_OUT")
        elif response.error_code == MoveItErrorCodes.START_STATE_IN_COLLISION:
            raise MPStartStateInCollisionError("MoveItErrorCodes -10 START_STATE_IN_COLLISION")
        elif response.error_code == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            raise MPStartStateInCollisionError("MoveItErrorCodes -11 START_STATE_VIOLATES_PATH_CONSTRAINTS")
        elif response.error_code == MoveItErrorCodes.GOAL_IN_COLLISION:
            raise MPTargetInCollisionError("MoveItErrorCodes -12 GOAL_IN_COLLISION")
        elif response.error_code == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            raise MPTargetInCollisionError("MoveItErrorCodes -13 GOAL_VIOLATES_PATH_CONSTRAINTS")
        elif response.error_code == MoveItErrorCodes.INVALID_GROUP_NAME:
            raise PlanningGroupNotExistsError("INVALID_GROUP_NAME -15 INVALID_GROUP_NAME")
        else:
            raise RosError(
                "MoveItErrorCodes {}, {}".format(response.error_code.human_readable, int(response.error_code)),
                response.error_code,
            )


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

    # We no longer return the Attached Collision Meshes in the trajectory object
    return trajectory

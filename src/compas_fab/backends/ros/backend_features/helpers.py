from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.ros.exceptions import RosError
from compas_fab.backends.ros.messages import Constraints
from compas_fab.backends.ros.messages import JointConstraint
from compas_fab.backends.ros.messages import MoveItErrorCodes
from compas_fab.backends.ros.messages import PositionConstraint
from compas_fab.backends.ros.messages import OrientationConstraint
from compas_fab.robots import Duration
from compas_fab.robots import JointTrajectoryPoint


def validate_response(response):
    """Raise an exception if the response indicates an error condition."""
    if response.error_code != MoveItErrorCodes.SUCCESS:
        raise RosError(response.error_code.human_readable,
                       int(response.error_code))


def convert_constraints_to_rosmsg(constraints, header):
    """Convert COMPAS FAB constraints into ROS Messages."""
    if not constraints:
        return None

    ros_constraints = Constraints()
    for c in constraints:
        if c.type == c.JOINT:
            ros_constraints.joint_constraints.append(
                JointConstraint.from_joint_constraint(c))
        elif c.type == c.POSITION:
            ros_constraints.position_constraints.append(
                PositionConstraint.from_position_constraint(header, c))
        elif c.type == c.ORIENTATION:
            ros_constraints.orientation_constraints.append(
                OrientationConstraint.from_orientation_constraint(header, c))
        else:
            raise NotImplementedError

    return ros_constraints


def convert_trajectory_points(points, types):
    result = []

    for pt in points:
        jtp = JointTrajectoryPoint(values=pt.positions,
                                   types=types,
                                   velocities=pt.velocities,
                                   accelerations=pt.accelerations,
                                   effort=pt.effort,
                                   time_from_start=Duration(pt.time_from_start.secs, pt.time_from_start.nsecs))

        result.append(jtp)

    return result

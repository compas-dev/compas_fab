import math
from typing import Optional

from compas_robots import Configuration
from compas_robots.model import Joint

from compas_fab.robots import RobotCell


def get_smaller_angle(angle):  # delete
    angle1 = angle % (2 * math.pi)
    angle2 = angle1 - 2 * math.pi
    return angle1 if angle1 < math.fabs(angle2) else angle2


def smallest_joint_angles(joint_angles):  # move to configuration ?
    return [get_smaller_angle(j) for j in joint_angles]


def fit_within_bounds(angle, lower, upper):
    while angle < lower:
        angle += 2 * math.pi
    while angle > upper:
        angle -= 2 * math.pi
    assert angle >= lower and angle <= upper, "Joint angle out of bounds."
    return angle


def try_to_fit_configurations_between_bounds(robot_cell: RobotCell, configurations: list[Configuration], group: Optional[str] = None) -> list[Configuration]:
    joints = robot_cell.get_configurable_joints(group=group)
    for i, c in enumerate(configurations):
        if c is None:
            continue
        try:
            fitted = []
            for angle, joint in zip(c.values(), joints):
                if joint.type == Joint.CONTINUOUS:
                    fitted.append(get_smaller_angle(angle))
                else:
                    fitted.append(fit_within_bounds(angle, joint.limit.lower, joint.limit.upper))
            configurations[i].joint_values = fitted
        except AssertionError:
            configurations[i] = None
    return configurations

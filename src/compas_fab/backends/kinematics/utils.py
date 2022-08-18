import math
from compas.robots import Configuration


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


def joint_angles_to_configurations(robot, solutions, group=None):
    joint_names = robot.get_configurable_joint_names(group=group)
    return [
        Configuration.from_revolute_values(q, joint_names=joint_names) if q else None
        for q in solutions
    ]


def try_to_fit_configurations_between_bounds(robot, configurations, group=None):
    j1, j2, j3, j4, j5, j6 = robot.get_configurable_joints(group=group)
    for i, c in enumerate(configurations):
        if c is None:
            continue
        a1, a2, a3, a4, a5, a6 = c.values()
        try:
            a1 = fit_within_bounds(a1, j1.limit.lower, j1.limit.upper)
            a2 = fit_within_bounds(a2, j2.limit.lower, j2.limit.upper)
            a3 = fit_within_bounds(a3, j3.limit.lower, j3.limit.upper)
            a4 = fit_within_bounds(a4, j4.limit.lower, j4.limit.upper)
            a5 = fit_within_bounds(a5, j5.limit.lower, j5.limit.upper)
            a6 = fit_within_bounds(a6, j6.limit.lower, j6.limit.upper)
            configurations[i].joint_values = [a1, a2, a3, a4, a5, a6]
        except AssertionError:
            configurations[i] = None
    return configurations

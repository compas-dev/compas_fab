import math
from compas.geometry import argmin

def get_smaller_angle(angle): # delete
    angle1 = angle % (2 * math.pi)
    angle2 = angle1 - 2 * math.pi
    return angle1 if angle1 < math.fabs(angle2) else angle2


def fit_within_bounds(angle, lower, upper):
    while angle < lower:
        angle += 2 * math.pi
    while angle > upper:
        angle -= 2 * math.pi
    assert(angle >= lower and angle <= upper)
    return angle


def find_closest_configuration(configurations, configuration): # should move to configurations ?
    diffs = []
    configs = []
    for c in configurations:
        if c:
            configs.append(c)
            diffs.append(sum([abs(d) for d in configuration.iter_differences(c)]))
    return argmin(diffs)
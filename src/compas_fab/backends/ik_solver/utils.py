import math


def get_smaller_angle(angle):
    while angle > 2 * math.pi:
        angle -= 2 * math.pi
    while angle < -2 * math.pi:
        angle += 2 * math.pi
    if math.fabs(angle - 2 * math.pi) < math.fabs(angle):
        return angle - 2 * math.pi
    elif math.fabs(angle + 2 * math.pi) < math.fabs(angle):
        return angle + 2 * math.pi
    else:
        return angle

def fit_within_bounds(angle, lower, upper):
    angle = get_smaller_angle(angle)
    if angle < lower:
        angle += 2 * math.pi
    if angle > upper:
        angle -= 2 * math.pi
    assert(angle >= lower and angle <= upper)
    return angle
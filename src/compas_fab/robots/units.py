from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math

__all__ = [
    'to_radians',
    'to_degrees',
]

def to_radians(degrees):
    """Converts a list of floats representing degrees to a list of radians.

    Parameters
    ----------
    degrees : list of :obj:`float`
        List of degrees.

    Returns
    -------
    list of :obj:`float`
        List of radians.
    """
    return [math.radians(d) for d in degrees]


def to_degrees(radians):
    """Converts a list of floats representing radians to a list of degrees.

    Parameters
    ----------
    radians : list of :obj:`float`
        List of radians.

    Returns
    -------
    list of :obj:`float`
        List of degress.
    """
    return [math.degrees(r) for r in radians]

"""Unit conversions on lists, useful for defining joint values."""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math

__all__ = [
    'to_radians',
    'to_degrees',
]


def to_radians(degrees):
    """Convert a list of floats representing degrees to a list of radians.

    Parameters
    ----------
    degrees : :obj:`list` of :obj:`float`
        List of angle values in degrees.

    Returns
    -------
    :obj:`list` of :obj:`float`
        List of angle values in radians.
    """
    return [math.radians(d) for d in degrees]


def to_degrees(radians):
    """Convert a list of floats representing radians to a list of degrees.

    Parameters
    ----------
    radians : :obj:`list` of :obj:`float`
        List of angle values in radians.

    Returns
    -------
    :obj:`list` of :obj:`float`
        List of degress.
    """
    return [math.degrees(r) for r in radians]

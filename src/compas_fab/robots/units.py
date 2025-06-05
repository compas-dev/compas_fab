"""Unit conversions on lists, useful for defining joint values."""

import math

__all__ = [
    "to_degrees",
    "to_radians",
]


def to_radians(degrees: list[float]) -> list[float]:
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


def to_degrees(radians: list[float]) -> list[float]:
    """Convert a list of floats representing radians to a list of degrees.

    Parameters
    ----------
    radians : :obj:`list` of :obj:`float`
        List of angle values in radians.

    Returns
    -------
    :obj:`list` of :obj:`float`
        List of degrees.
    """
    return [math.degrees(r) for r in radians]

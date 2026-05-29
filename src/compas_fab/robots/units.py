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
    degrees : `list` of `float`
        List of angle values in degrees.

    Returns
    -------
    `list` of `float`
        List of angle values in radians.
    """
    return [math.radians(d) for d in degrees]


def to_degrees(radians: list[float]) -> list[float]:
    """Convert a list of floats representing radians to a list of degrees.

    Parameters
    ----------
    radians : `list` of `float`
        List of angle values in radians.

    Returns
    -------
    `list` of `float`
        List of degrees.
    """
    return [math.degrees(r) for r in radians]

import math

from compas import IPY

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import List  # noqa: F401
        from typing import Optional  # noqa: F401

__all__ = [
    "allclose",
    "arange",
    "argmax",
    "argmin",
    "argsort",
    "clamp",
    "diffs",
    "map_range",
    "range_geometric_row",
    "sign",
]


def map_range(value, from_min, from_max, to_min, to_max):
    # type: (float, float, float, float, float) -> float
    """Performs a linear interpolation of a value within the range of [from_min,
    from_max] to another range of [to_min, to_max].

    Parameters
    ----------
    value : :obj:`float`
        The value to map.
    from_min : :obj:`float`
        The minimum value of the input range.
    from_max : :obj:`float`
        The maximum value of the input range.
    to_min : :obj:`float`
        The minimum value of the output range.
    to_max : :obj:`float`
        The maximum value of the output range.

    Returns
    -------
    :obj:`float`
        The mapped value.
    """
    from_range = from_max - from_min
    to_range = to_max - to_min
    value_scaled = (value - from_min) / float(from_range)
    return to_min + (value_scaled * to_range)


def range_geometric_row(number, d, r=1.1):
    # type: (float, int, float) -> List[float]
    """Returns a list of numbers with a certain relation to each other.

    The function divides one number into a list of d numbers [n0, n1, ...], such
    that their sum is number and the relation between the numbers is defined
    with n1 = n0 / r, n2 = n1 / r, n3 = n2 / r, ...

    Parameters
    ----------
    number : :obj:`float`
        The initial number to divide. This number is the first element
        in the returned list.
    d : :obj:`int`
        The number of elements in the returned list.
    r : :obj:`float`, optional
        The ratio between the numbers in the list. Default is 1.1.

    Returns
    -------
    :obj:`list` of :obj:`float`
        The list of numbers with the geometric relation.
    """
    if r <= 0:
        raise ValueError("r must be > 0")

    n0 = number / ((1 - (1 / r) ** d) / (1 - 1 / r))

    numbers = [n0]
    for i in range(d - 1):
        numbers.append(numbers[-1] / r)
    return numbers


def arange(start, stop, step):
    # type: (float, float, float) -> List[float]
    """Returns evenly spaced values within a given interval.

    The function is similar to NumPy's *arange* function.

    Parameters
    ----------
    start : :obj:`float`
        The starting value of the sequence.
    stop : :obj:`float`
        The end value of the sequence.
    step : :obj:`float`
        The spacing between the values.

    Returns
    -------
    :obj:`list` of :obj:`float`
        The list of evenly spaced values.
    """
    if math.fabs(stop - (start + step)) > math.fabs(stop - start):
        raise ValueError("Please check the sign of step.")

    len = int(math.ceil((stop - start) / float(step)))
    return [start + i * step for i in range(len)]


def diffs(l1, l2):
    # type: (List[float], List[float]) -> List[float]
    """Returns the element-wise differences between two lists.

    Parameters
    ----------
    l1 : :obj:`list` of :obj:`float`
        The first list.
    l2 : :obj:`list` of :obj:`float`
        The second list.

    Returns
    -------
    :obj:`list` of :obj:`float`
        The element-wise differences between the two lists.

    Raises
    ------
    ValueError
        If 2 lists of different length are passed.
    """
    if len(l1) != len(l2):
        raise ValueError("Two lists must be of equal length.")
    return [math.fabs(a - b) for a, b in zip(l1, l2)]


def allclose(l1, l2, tol=1e-05):
    # type: (List[float], List[float], Optional[float]) -> bool
    """Returns True if two lists are element-wise equal within a tolerance.

    The function is similar to NumPy's *allclose* function.

    Parameters
    ----------
    l1 : :obj:`list` of :obj:`float`
        The first list.
    l2 : :obj:`list` of :obj:`float`
        The second list.
    tol : :obj:`float`, optional
        The tolerance within which the lists are considered equal. Default is 1e-05.

    Returns
    -------
    bool
        True if the lists are equal within the tolerance, otherwise False.

    Raises
    ------
    ValueError
        If 2 lists of different length are passed.
    """

    if len(l1) != len(l2):
        raise ValueError("Two lists must be of equal length.")

    for a, b in zip(l1, l2):
        if math.fabs(a - b) > tol:
            return False
    return True


def argsort(numbers):
    # type: (List[float]) -> List[int]
    """Returns the indices that would sort an array of numbers.

    The function is similar to NumPy's *argsort* function.

    Parameters
    ----------
    numbers : :obj:`list` of :obj:`float`
        A list of numbers.

    Returns
    -------
    :obj:`list` of :obj:`int`
        The indices that would sort the array.

    Notes
    -----
    For a large list of numbers reconsider using NumPy's *argsort* function,
    since this function might take too long.

    """
    return [i[0] for i in sorted(enumerate(numbers), key=lambda x: x[1])]


def argmin(numbers):
    # type: (List[float]) -> int
    """Returns the index of the minimum value in numbers.

    The function is similar to NumPy's *argmin* function.

    Parameters
    ----------
    numbers : :obj:`list` of :obj:`float`
        A list of numbers.

    Returns
    -------
    :obj:`int`
        The index of the minimum value in the list.

    Notes
    -----
    For a large list of numbers reconsider using NumPy's *argmin* function,
    since this function might take too long.
    """
    return argsort(numbers)[0]


def argmax(numbers):
    # type: (List[float]) -> int
    """Returns the index of the maximum value in numbers.

    The function is similar to NumPy's *argmax* function.

    Parameters
    ----------
    numbers : :obj:`list` of :obj:`float`
        A list of numbers.

    Returns
    -------
    :obj:`int`
        The index of the maximum value in the list.

    Notes
    -----
    For a large list of numbers reconsider using NumPy's *argmax* function,
    since this function might take too long.
    """
    return argsort(numbers)[-1]


def sign(number):
    # type: (float) -> int
    """Returns the sign of a number: +1 or -1.

    Parameters
    ----------
    number : float
        A number to check the sign of.

    Returns
    -------
    int
        `+1` if the number has positive sign,
        `0` if the number is zero,
        `-1` if the number has negative sign.
    """
    return int(int((number) > 0) - int((number) < 0))


def clamp(value, min_value, max_value):
    # type: (float, float, float) -> float
    """Clamps a value within the bound [min_value, max_value]

    Parameters
    ----------
    value : float
        The value to clamp.
    min_value : float
        The minimum value.
    max_value : float
        The maximum value.

    Returns
    -------
    float
        The clamped value.
    """
    if min_value > max_value:
        raise ValueError("min_value must be bigger than max_value")
    return float(min(max(value, min_value), max_value))


if __name__ == "__main__":
    print(map_range(2, 0, 10, 0, 100))
    print(arange(3, -4, -0.2))
    print(argsort([34, 1, 7, 2, 100]))
    print(clamp(5, 1, 4))
    print(clamp(0, 1, 4))
    print(clamp(3, 1, 4))

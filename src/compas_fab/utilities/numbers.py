import math

__all__ = [
    'map_range',
    'range_geometric_row',
    'arange',
    'diffs',
    'allclose',
    'argsort',
    'argmin',
    'argmax',
    'clamp',
]


def map_range(value, from_min, from_max, to_min, to_max):
    """Performs a linear interpolation of a value within the range of [from_min,
        from_max] to another range of [to_min, to_max].
    """
    from_range = from_max - from_min
    to_range = to_max - to_min
    value_scaled = (value - from_min) / float(from_range)
    return to_min + (value_scaled * to_range)


def range_geometric_row(number, d, r=1.1):
    """Returns a list of numbers with a certain relation to each other.

    The function divides one number into a list of d numbers [n0, n1, ...], such
    that their sum is number and the relation between the numbers is defined
    with n1 = n0 / r, n2 = n1 / r, n3 = n2 / r, ...
    """
    if r <= 0:
        raise ValueError("r must be > 0")

    n0 = number / ((1 - (1 / r)**d) / (1 - 1 / r))

    numbers = [n0]
    for i in range(d - 1):
        numbers.append(numbers[-1] / r)
    return numbers


def arange(start, stop, step):
    """Returns evenly spaced values within a given interval.

    The function is similar to NumPy's *arange* function.
    """
    if math.fabs(stop - (start + step)) > math.fabs(stop - start):
        raise ValueError("Please check the sign of step.")

    len = int(math.ceil((stop - start) / float(step)))
    return [start + i * step for i in range(len)]


def diffs(l1, l2):
    """Returns the element-wise differences between two lists.

    Raises
    ------
    ValueError
        If 2 lists of different length are passed.
    """
    if len(l1) != len(l2):
        raise ValueError("Pass 2 lists of equal length.")
    return [math.fabs(a - b) for a, b in zip(l1, l2)]


def allclose(l1, l2, tol=1e-05):
    """Returns True if two lists are element-wise equal within a tolerance.

    The function is similar to NumPy's *allclose* function.
    """
    for a, b in zip(l1, l2):
        if math.fabs(a - b) > tol:
            return False
    return True


def argsort(numbers):
    """Returns the indices that would sort an array of numbers.

    The function is similar to NumPy's *argsort* function.

    Note
    ----
    For a large list of numbers reconsider using NumPy's *argsort* function,
    since this function might take too long.
    """
    return [i[0] for i in sorted(enumerate(numbers), key=lambda x:x[1])]


def argmin(numbers):
    """Returns the index of the minimum value in numbers.

    The function is similar to NumPy's *argmin* function.

    Note
    ----
    For a large list of numbers reconsider using NumPy's *argmin* function,
    since this function might take too long.
    """
    return argsort(numbers)[0]


def argmax(numbers):
    """Returns the index of the maximum value in numbers.

    The function is similar to NumPy's *argmax* function.

    Note
    ----
    For a large list of numbers reconsider using NumPy's *argmax* function,
    since this function might take too long.
    """
    return argsort(numbers)[-1]


def clamp(value, min_value, max_value):
    """Clamps a value witin the bound [min_value, max_value]

    Returns
    -------
    float
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

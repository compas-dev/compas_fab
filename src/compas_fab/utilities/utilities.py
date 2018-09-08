from __future__ import print_function

__all__ = [
    'sign',
    'argsort'
]


def sign(number):
    """Returns the sign of a number: +1 or -1.
    """
    return int(int((number) > 0) - int((number) < 0))


def argsort(numbers):
    """Returns the indices that would sort a list of numbers.
    """
    return [i for i, _v in sorted(enumerate(numbers), key=lambda x: x[1])]


if __name__ == "__main__":
    numbers = [1, 5, 7, 2, 0]
    print(argsort(numbers))

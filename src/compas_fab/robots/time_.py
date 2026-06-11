from typing import Union

from compas.data import Data

__all__ = [
    "Duration",
]


class Duration(Data):
    """Duration is used to accurately describe the passage of time.
    It consists of seconds and nanoseconds, the total duration is the sum of the two values.

    Parameters
    ----------
    secs
        Integer representing number of seconds.
        If a float is passed, the integer portion is assigned to secs and
        the decimal portion of the secs variable is converted and added to nsecs.
    nsecs
        Integer representing number of nanoseconds.

    Attributes
    ----------
    seconds
        Returns the total duration as floating-point seconds.
    secs
        Float representing number of seconds.
    nsecs
        Integer representing number of nanoseconds.

    Examples
    --------
    >>> d = Duration(2, 5e8)
    >>> d.seconds
    2.5
    >>> d = Duration(2.6, 0)
    >>> d.seconds
    2.6
    >>> d = Duration(2.6, 5e8)
    >>> d.secs
    3
    >>> d.nsecs
    100000000
    >>> d.seconds
    3.1

    """

    def __init__(self, secs: Union[int, float], nsecs: int) -> None:
        super(Duration, self).__init__()
        sec_to_nano_factor = 1e9
        quotient, remainder = divmod(secs, 1)

        self.secs = int(quotient)
        self.nsecs = int(remainder * sec_to_nano_factor) + int(nsecs)

        # If nsecs is greater than 1 second, add the remainder back to secs
        if self.nsecs >= sec_to_nano_factor:
            self.secs += 1
            self.nsecs -= int(sec_to_nano_factor)

    def __str__(self):
        return "Duration({!r}, {!r})".format(self.secs, self.nsecs)

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        if isinstance(other, Duration):
            return self.seconds == other.seconds

        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    @property
    def seconds(self) -> float:
        return self.secs + 1e-9 * self.nsecs

    @property
    def __data__(self):
        """`dict` : The data representing the duration."""
        return {"secs": self.secs, "nsecs": self.nsecs}

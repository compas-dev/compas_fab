from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.data import Data

__all__ = [
    "Duration",
]


class Duration(Data):
    """Duration consists of two values: seconds (float) and nanoseconds (int).
    The total number of seconds is the sum of these values.
    The decimal portion of the secs variable is converted to an integer and added to nsecs.

    Attributes
    ----------
    secs: float
        Float representing number of seconds.
    nsecs: int
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

    def __init__(self, secs, nsecs):
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
    def seconds(self):
        """Returns the duration as floating-point seconds.

        Returns
        -------
        float
            Floating-point seconds
        """
        return self.secs + 1e-9 * self.nsecs

    @property
    def __data__(self):
        """:obj:`dict` : The data representing the duration."""
        return {"secs": self.secs, "nsecs": self.nsecs}

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
    """

    def __init__(self, secs, nsecs):
        super(Duration, self).__init__()
        sec_to_nano_factor = 1e9
        quotient, remainder = divmod(secs, 1)

        self.secs = int(quotient)
        self.nsecs = int(remainder * sec_to_nano_factor) + int(nsecs)

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

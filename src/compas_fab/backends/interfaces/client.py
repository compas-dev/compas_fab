from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas


if compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.robots import Robot


class ClientInterface(object):
    """Interface for implementing backend clients.

    Attributes
    ----------
    robot : :class:`compas_fab.robots.Robot`, read-only
        The robot instance associated with the client.
    """

    def __init__(self):
        self._robot = None  # type: Robot

    @property
    def robot(self):
        # type: () -> Robot
        return self._robot

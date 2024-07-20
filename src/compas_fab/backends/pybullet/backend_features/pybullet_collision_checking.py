from itertools import combinations

import compas

from compas.geometry import Frame
from compas_fab.backends.interfaces import CheckCollision
from compas_fab.backends import CollisionCheckInCollisionError

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas_fab.robots import Robot  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401

    import pybullet
from compas_fab.backends.pybullet.const import STATIC_MASS


class PyBulletCheckCollision(CheckCollision):

    # =======================================
    def check_collision(self, robot, configuration=None):
        pass

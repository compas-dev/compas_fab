from compas_fab.backends.interfaces import SetRobotCell

import compas
from compas.geometry import Frame

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

from compas_fab.backends.pybullet.const import STATIC_MASS


class AnalyticalSetRobotCell(SetRobotCell):

    def set_robot_cell(self, robot_cell, robot_cell_state=None, options=None):
        # type: (RobotCell, Optional[RobotCellState], Optional[Dict]) -> None
        """Pass the models in the robot cell to the Analytical Planner.

        The planner will use the tool information for frame transformation.

        Note that the Analytical Planner does not support collision checking. Therefore, the
        geometry of the robot, tools and rigid bodies in the robot cell are not checked.

        Note
        ----
        The robot_cell_state and options are not used in this implementation.

        """
        # Update the robot cell in the client
        self._robot_cell = robot_cell

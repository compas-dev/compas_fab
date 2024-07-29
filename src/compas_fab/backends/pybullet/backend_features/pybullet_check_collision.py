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
        from compas_fab.backends import PyBulletPlanner  # noqa: F401

    import pybullet
from compas_fab.backends.pybullet.const import STATIC_MASS


class PyBulletCheckCollision(CheckCollision):

    # =======================================
    def check_collision(self, state, group=None, options=None):
        # type: (RobotCellState, Optional[str], Optional[Dict]) -> None
        """Check if the robot is in collision.

        The collision check involves the following steps:

        1. Check for collisions between each robot link.
        2. Check for collisions between each robot link and each tool.
        3. Check for collisions between each robot link and each rigid body.
        4. Check for collisions between each attached rigid body and all other rigid bodies.
        5. Check for collisions between each tool and each rigid body.

        In each of the above steps, the collision check is skipped if the collision is allowed
        by the semantics of the robot, tool, or rigid body. For details, see in-line comments in code.


        A collision report is returned with the CollisionCheckError exception message.
        Typically, the exception is raised on the first collision detected and the rest of
        the collision pairs are not checked.
        If ``full_report`` is set to ``True``, the exception will be raised after all
        collision pairs are checked, and the report will contain all failing collision pairs.
        This can be useful for debugging and visualization.

        Parameters
        ----------
        state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
            The attribute `robot_configuration`, must contain the full configuration of the robot corresponding to the planning group.
        group : str, optional
            The planning group used for collision checking. Defaults to the robot's main planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"verbose"``: (:obj:`bool`, optional) When ``True``, additional information is printed.
              Defaults to ``False``
            - ``"full_report"``: (:obj:`bool`, optional) When ``True``, all collision pairs are checked
              even when one encountered a collision. Defaults to ``False``

        Raises
        ------
        CollisionCheckInCollisionError
            If the robot is in collision.
        """
        options = options or {}

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot = client.robot  # type: Robot

        # Set the robot cell state
        planner.set_robot_cell_state(state)

        # Call the collision checking function from client
        verbose = options.get("verbose", False)
        full_report = options.get("full_report", False)
        client.check_collisions(verbose=verbose, full_report=full_report)

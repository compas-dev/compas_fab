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


class PyBulletSetRobotCell(SetRobotCell):

    def set_robot_cell(self, robot_cell, robot_cell_state=None, options=None):
        # type: (RobotCell, Optional[RobotCellState], Optional[Dict]) -> None
        """Pass the models in the robot cell to the Pybullet client.

        The client keeps the robot cell models in memory and uses them for planning.
        Calling this method will override the previous robot cell in the client.
        It should be called only if the robot cell models have changed.

        """
        client = self.client  # type: PyBulletClient

        # TODO: Check for new, modified and removed objects compared to the
        # TODO: previous robot cell state and update the PyBullet world accordingly

        # At the moment, all previous object in the PyBullet world are removed
        # and the new robot cell is added to the PyBullet world

        # Remove all objects from the PyBullet world
        # client.remove_all_tools()
        # client.remove_all_rigid_bodies()

        # Add the robot cell to the PyBullet world
        for name, tool_model in robot_cell.tool_models.items():
            client.add_tool(name, tool_model)
        for name, rigid_body in robot_cell.rigid_body_models.items():
            client.add_rigid_body(name, rigid_body)
            # client.convert_mesh_to_body(rigid_body.visual_meshes[0], Frame.worldXY())
        # Update the robot cell in the client
        self._robot_cell = robot_cell

        # If a robot cell state is provided, update the client's robot cell state
        # if robot_cell_state:
        #     self.set_robot_cell_state(robot_cell_state, options)

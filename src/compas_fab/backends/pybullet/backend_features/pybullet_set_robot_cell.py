from compas_fab.backends.interfaces import SetRobotCell

import compas


if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401

        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401


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
        # previous_robot_cell = client.robot_cell or RobotCell()

        # At the moment, all previous object in the PyBullet world are removed
        # and the new robot cell is added to the PyBullet world

        # Remove all objects from the PyBullet world

        for tool_id in list(client.tools_puids.keys()):
            client.remove_tool(tool_id)
        # client.tools_puids = {}
        for rigid_body_id in list(client.rigid_bodies_puids.keys()):
            client.remove_rigid_body(rigid_body_id)
        # client.rigid_bodies_puids = {}

        # Add the robot cell to the PyBullet world
        for name, tool_model in robot_cell.tool_models.items():
            client.add_tool(name, tool_model)
        for name, rigid_body in robot_cell.rigid_body_models.items():
            client.add_rigid_body(name, rigid_body)
            # client.convert_mesh_to_body(rigid_body.visual_meshes[0], Frame.worldXY())

        # Feed the robot to the client
        if robot_cell.robot:
            robot_cell.robot.ensure_geometry()
            robot_cell.robot.ensure_semantics()
            client.set_robot(robot_cell.robot)

        # Keep a copy of the robot cell in the client
        client._robot_cell = robot_cell.copy()

        # If a robot cell state is provided, update the client's robot cell state
        if robot_cell_state:
            self.set_robot_cell_state(robot_cell_state)

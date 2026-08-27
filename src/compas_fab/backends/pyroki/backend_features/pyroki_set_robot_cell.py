from copy import deepcopy

from compas_fab.backends.interfaces import SetRobotCell

from ..model import robot_model_to_pyroki


class PyRokiSetRobotCell(SetRobotCell):
    """Build and cache a direct PyRoKI representation of a FAB robot cell."""

    def set_robot_cell(self, robot_cell, robot_cell_state=None, options=None):
        """Store a robot cell and build its direct PyRoKI kinematic bridge."""
        self.client._robot_cell = deepcopy(robot_cell)
        self.client._pyroki_model = robot_model_to_pyroki(self.client.robot_model)
        self.client._problem_cache.clear()
        if robot_cell_state is not None:
            self.set_robot_cell_state(robot_cell_state)

from compas_fab.backends.interfaces import SetRobotCellState


class PyRokiSetRobotCellState(SetRobotCellState):
    """Store the FAB state used to seed numerical optimization."""

    def set_robot_cell_state(self, robot_cell_state, options=None):
        """Store the current FAB cell state by reference."""
        self.client._robot_cell_state = robot_cell_state

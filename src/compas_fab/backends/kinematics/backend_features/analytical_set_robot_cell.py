from typing import Optional

from copy import deepcopy

from compas_fab.backends.interfaces import SetRobotCell
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState


class AnalyticalSetRobotCell(SetRobotCell):
    def set_robot_cell(self, robot_cell : RobotCell, robot_cell_state : RobotCellState = None, options : Optional[dict] = None):
        """Pass the models in the robot cell to the Analytical Planner.

        The planner will use the tool information for frame transformation.

        Note that the Analytical Planner does not support collision checking. Therefore, the
        geometry of the robot, tools and rigid bodies in the robot cell are not checked.

        Attributes
        ----------
        robot_cell : :class:`compas_fab.robots.RobotCell`
            The robot cell to set for the planner.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
            The robot cell state to set for the planner.
        options : dict, optional
            This input is not used by the AnalyticalClient.

        Notes
        -----
        If the robot_cell_state is provided, it will be stored in the client for later use.
        """
        # Update the robot cell in the client
        self.client._robot_cell = deepcopy(robot_cell)

        if robot_cell_state:
            self.client._robot_cell_state = self.set_robot_cell_state(robot_cell_state)

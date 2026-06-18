from compas_fab.backends.interfaces import SetRobotCellState
from compas_fab.robots import RobotCellState


class AnalyticalSetRobotCellState(SetRobotCellState):
    def set_robot_cell_state(self, robot_cell_state: RobotCellState):
        """The Analytical Planner does not have collision checking ability, therefore it does not really need
        the robot cell state. This function simply serves as the landing point for other BackendFeature to
        safely call `planner.set_robot_cell_state` without having to check if the planner actually supports it.

        The given RobotCellState object is stored on the client by reference.
        Pass ``robot_cell_state.copy()`` if you intend to mutate the object after this call.

        """
        # Update the robot cell in the client
        self.client._robot_cell_state = robot_cell_state

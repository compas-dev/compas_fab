from compas_fab.backends.exceptions import BackendError
from compas_fab.backends.interfaces import ForwardKinematics


import compas

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import AnalyticalKinematicsPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas.geometry import Frame
        from typing import Dict, List, Optional, Tuple  # noqa: F401


class AnalyticalForwardKinematics(ForwardKinematics):
    """Backend feature to calculate the robot's forward kinematics for a given joint configuration.

    This mixin is used by AnalyticalKinematicsPlanner.
    The kinematical solver in the planner must implement the method ``forward``.

    """

    def forward_kinematics(self, robot_cell_state, group=None, options=None):
        # type: (RobotCellState, Optional[str], Optional[dict]) -> Frame
        """Calculate the forward kinematics for a given joint configuration."""
        planner = self  # type: AnalyticalKinematicsPlanner
        robot = self.robot_cell.robot  # type: Robot

        if group is not None:
            raise ValueError("AnalyticalKinematicsPlanner does not support groups")
        if options and "link" in options:
            raise ValueError("AnalyticalKinematicsPlanner does not support the 'link' option")

        joint_values = robot_cell_state.robot_configuration.joint_values
        frame = planner.kinematics_solver.forward(joint_values)

        # Scale the frame to user units
        frame = self._scale_output_frame(frame)

        return frame

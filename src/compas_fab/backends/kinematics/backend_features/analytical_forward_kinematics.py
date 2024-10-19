from compas import IPY

from compas_fab.backends.interfaces import ForwardKinematics

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401

        from compas.geometry import Frame  # noqa: F401

        from compas_fab.backends import AnalyticalKinematicsPlanner  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import TargetMode  # noqa: F401


class AnalyticalForwardKinematics(ForwardKinematics):
    """Backend feature to calculate the robot's forward kinematics for a given joint configuration.

    This mixin is used by AnalyticalKinematicsPlanner.
    The kinematical solver in the planner must implement the method ``forward``.

    """

    def forward_kinematics(self, robot_cell_state, target_mode, scale=None, group=None, options=None):
        # type: (RobotCellState, TargetMode | str, Optional[float], Optional[str], Optional[dict]) -> Frame
        """Calculate the forward kinematics for a given joint configuration."""
        planner = self  # type: AnalyticalKinematicsPlanner
        robot_cell = self.client.robot_cell  # type: RobotCell

        if group is not None and group != robot_cell.main_group_name:
            # NOTE: Analytical IK must operate on all the joints as defined in the KinematicsSolver
            # There is a chance that the planning group specified by the URDF does not match with these joints
            # At the moment there is no automatic check for this except to assume that the main group
            # is the one to be used.
            raise ValueError("AnalyticalKinematicsPlanner only support the main planning group")
        if options and "link" in options:
            raise ValueError("AnalyticalKinematicsPlanner does not support the 'link' option")

        joint_values = robot_cell_state.robot_configuration.joint_values
        pcf_frame = planner.kinematics_solver.forward(joint_values)

        target_frame = robot_cell.pcf_to_target_frames(robot_cell_state, pcf_frame, target_mode, group)

        # Scale the frame to user units
        if scale:
            target_frame.scale(scale)

        return target_frame

    def forward_kinematics_to_link(self, robot_cell_state, link_name, scale=None, group=None, options=None):
        # type: (RobotCellState, str, Optional[float], Optional[str], Optional[dict]) -> Frame
        """Calculate the forward kinematics for a given joint configuration to a specific link.

        This method is not supported by AnalyticalKinematicsPlanner.
        """
        raise NotImplementedError("AnalyticalKinematicsPlanner does not support the 'link' option")

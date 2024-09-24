from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import ForwardKinematics
from compas_fab.backends.exceptions import PlanningGroupNotExistsError
from compas_fab.robots import TargetMode
import compas

if compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401


class PyBulletForwardKinematics(ForwardKinematics):
    """Mix-in function to calculate the robot's forward kinematic."""

    def forward_kinematics(self, robot_cell_state, target_mode, group=None, scale=None, options=None):
        # type: (RobotCellState, TargetMode | str, Optional[str], Optional[float], Optional[dict]) -> Frame
        """Calculate the target frame of the robot from the provided RobotCellState.

        The function can return the planner coordinate frame (PCF), the tool coordinate frame (TCF),
        or the workpiece's object coordinate frame (OCF) based on the ``target_mode`` provided.

        - ``"Target.ROBOT"`` will return the planner coordinate frame (PCF).
        - ``"Target.TOOL"`` will return the tool coordinate frame (TCF) if a tool is attached.
        - ``"Target.WORKPIECE"`` will return the workpiece's object coordinate frame (OCF)
          if a workpiece is attached.

        Collision checking is not performed during the calculation. Consider using the
        :meth:`compas_fab.backends.PyBulletCheckCollision.check_collision` method to check for collisions.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
            The attribute `robot_configuration`, must contain the full configuration of the robot corresponding to the planning group.
            The Configuration object must include ``joint_names``.
            The robot cell state should also reflect the attachment of tools, if any.
        target_mode : :class:`compas_fab.robots.TargetMode` or str
            The target mode to select which frame to return.
        group : str, optional
            The planning group of the robot.
            Defaults to the robot's main planning group.
        scale : float, optional
            The scaling factor to apply to the resulting frame.
            For example, use ``'1000.0'`` to convert the result to millimeters.
            Defaults to None, which means no scaling is applied.
        options : dict, optional
            Dictionary for passing planner specific options.
            Currently unused.


        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).

        Raises
        ------
        :class:`compas_fab.backends.TargetModeMismatchError`
            If the selected TargetMode is not possible with the provided robot cell state.

        """
        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot = client.robot  # type: Robot
        group = group or robot.main_group_name

        # Check if the target mode is valid for the robot cell state
        planner.ensure_robot_cell_state_supports_target_mode(robot_cell_state, target_mode, group)

        # Check if the planning group is supported by the planner
        if group not in robot.group_names:
            raise PlanningGroupNotExistsError("Planning group '{}' is not supported by PyBullet planner.".format(group))

        # Setting the entire robot cell state, including the robot configuration
        planner.set_robot_cell_state(robot_cell_state)

        # Retrieve the PCF of the group
        link_name = robot.get_end_effector_link_name(group)
        link_id = client.robot_link_puids[link_name]
        pcf_frame = client._get_link_frame(link_id, client.robot_puid)

        # If no link name provided, and a tool is attached to the group, return the tool tip frame of the tool
        target_frame = planner.pcf_to_target_frames(pcf_frame, target_mode=target_mode, group=group)

        # Scale resulting frame to user units
        if scale:
            target_frame.scale(scale)

        return target_frame

    def forward_kinematics_to_link(self, robot_cell_state, link_name=None, group=None, scale=None, options=None):
        # type: (RobotCellState, Optional[str], Optional[str], Optional[float], Optional[dict]) -> Frame
        """Calculate the frame of the specified robot link from the provided RobotCellState.

        This function operates similar to :meth:`compas_fab.backends.PyBulletForwardKinematics.forward_kinematics`,
        but allows the user to specify which link to return. The function will return the frame of the specified
        link relative to the world coordinate frame (WCF).

        This can be convenient in scenarios where user objects (such as a camera) are attached to one of the
        robot's links and the user needs to know the position of the object relative to the world coordinate frame.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
        link_name : str, optional
            The name of the link to calculate the forward kinematics for.
            Defaults to the last link of the provided planning group.
        group : str, optional
            The planning group of the robot.
            Defaults to the robot's main planning group.
        scale : float, optional
            The scaling factor to apply to the resulting frame.
            For example, use ``'1000.0'`` to convert the result to millimeters.
            Defaults to None, which means no scaling is applied.
        options : dict, optional
            Dictionary for passing planner specific options.
            Currently unused.
        """
        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        robot = client.robot  # type: Robot
        group = group or robot.main_group_name

        # Check if the planning group is supported by the planner
        if group not in robot.group_names:
            raise PlanningGroupNotExistsError("Planning group '{}' is not supported by PyBullet planner.".format(group))

        # Setting the entire robot cell state, including the robot configuration
        planner.set_robot_cell_state(robot_cell_state)

        # Retrieve the PCF of the group
        link_id = client.robot_link_puids[link_name]
        link_frame = client._get_link_frame(link_id, client.robot_puid)

        # Scale resulting frame to user units
        if scale:
            link_frame.scale(scale)

        return link_frame

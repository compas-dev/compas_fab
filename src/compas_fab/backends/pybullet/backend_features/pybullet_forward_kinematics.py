from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.interfaces import ForwardKinematics
from compas_fab.utilities import from_tcf_to_t0cf

import compas

if compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from typing import Optional  # noqa: F401


class PyBulletForwardKinematics(ForwardKinematics):
    """Callable to calculate the robot's forward kinematic."""

    def forward_kinematics(self, robot_cell_state, group=None, options=None):
        # type: (RobotCellState, Optional[str], Optional[dict]) -> Frame
        """Calculate the robot's forward kinematic (FK).

        If no tool is attached to the robot, the frame of the end effector link (T0CF) will be returned.
        If a tool is attached to the robot, the frame of the tool tip (TCF) will be returned.
        If ``"link"`` is provided in the options, the frame of the specified link will be returned.

        Collision checking can be enabled by setting ``"check_collision"`` to ``True`` in the options.
        This will cause the backend to perform collision checking on the robot's configuration and raise
        a :class:`CollisionError` if the robot is in collision. This is equivalent to calling
        :meth:`compas_fab.backends.PyBulletClient.check_collisions`.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state describing the robot cell.
            The attribute `robot_configuration`, must contain the full configuration of the robot corresponding to the planning group.
            The Configuration object must include ``joint_names``.
            The robot cell state should also reflect the attachment of tools, if any.
            If a tool is attached to the robot, the tool coordinate frame (TCF) will be returned.
        group : str, optional
            The planning group used for determining the end effector and labeling
            the ``configuration``. Defaults to the robot's main planning group.
        options : dict, optional
            Dictionary containing the following key-value pairs:

            - ``"link"``: (:obj:`str`, optional) The name of the link to
              calculate the forward kinematics for. Defaults to the end effector.
            - ``"check_collision"``: (:obj:`str`, optional) When ``True``,
              :meth:`compas_fab.backends.PyBulletClient.check_collisions` will be called.
              Defaults to ``False``.

        Returns
        -------
        :class:`Frame`
            The frame in the world's coordinate system (WCF).

        Raises
        ------
        CollisionError
            If the configuration is in collision. Includes both self-collision and
            collision with the environment.

        """
        options = options or {"link": None, "check_collision": False}
        configuration = robot_cell_state.robot_configuration

        client = self.client  # type: PyBulletClient # Trick to keep intellisense happy
        planner = self  # type: PyBulletPlanner
        robot = client.robot
        group = group or robot.main_group_name
        cached_robot_model = client.get_cached_robot_model(robot)
        robot_body_id = client.get_uid(cached_robot_model)

        # Setting the entire robot cell state, including the robot configuration
        planner.set_robot_cell_state(robot_cell_state)

        # Check for collisions if requested, it will throw an exception if the robot is in collision
        if options.get("check_collision"):
            client.check_collisions(robot, configuration)

        # If a link name provided, return the frame of that link
        link_name = options.get("link")
        if link_name:
            if link_name in robot.get_link_names(group):
                raise KeyError("Link name provided is not part of the group")
            link_id = client._get_link_id_by_name(link_name, cached_robot_model)
            fk_frame = client._get_link_frame(link_id, robot_body_id)

        else:
            link_name = robot.get_end_effector_link_name(group)
            link_id = client._get_link_id_by_name(link_name, cached_robot_model)
            fk_frame = client._get_link_frame(link_id, robot_body_id)

            # If no link name provided, and a tool is attached to the group, return the tool tip frame of the tool
            robot_cell = planner.robot_cell  # type: RobotCell
            if robot_cell:
                attached_tool = robot_cell.get_attached_tool(robot_cell_state, group)
                if attached_tool:
                    # Do not scaling attached_tool.frame because Tools are modelled in meter scale
                    fk_frame = from_tcf_to_t0cf(fk_frame, attached_tool.frame)

        # Scale resulting frame to user units
        if robot.need_scaling:
            fk_frame = fk_frame.scaled(robot.scale_factor)

        return fk_frame

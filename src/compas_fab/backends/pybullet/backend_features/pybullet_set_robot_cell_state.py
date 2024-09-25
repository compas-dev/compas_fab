from compas import IPY
from compas.geometry import Frame
from compas.geometry import Transformation

from compas_fab.backends.interfaces import SetRobotCellState

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from compas_robots import ToolModel  # noqa: F401

        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.backends import PyBulletPlanner  # noqa: F401
        from compas_fab.robots import RigidBodyState  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import ToolState  # noqa: F401


class PyBulletSetRobotCellState(SetRobotCellState):
    def set_robot_cell_state(self, robot_cell_state):
        # type: (RobotCellState) -> None
        """Change the state of the models in the robot cell that have already been set to the Pybullet client.

        This function is typically called by planning functions that accepts a start state as input.
        The user should not need to call this function directly.

        The robot cell state must match the robot cell set earlier by :meth:`set_robot_cell`.

        If the robot_configuration is provided, the robot's configuration is updated and the
        position of the attached tools and rigid bodies are also updated accordingly.
        Otherwise, their positions are left unchanged.

        For stationary (non-attached) tools and rigid bodies, their positions are updated according to the
        provided frames in the tool_state and rigid_body_state respectively.

        Hidden tools and rigid bodies are not updated.

        Notes
        -----
        All the magic for transforming the attached objects happens here.
        """
        client = self.client  # type: PyBulletClient # Trick to keep intellisense happy
        robot_cell = client.robot_cell  # type: RobotCell
        planner = self  # type: PyBulletPlanner # noqa: F841

        # Check if the robot cell state matches the robot cell.
        # However if the robot is the only element in the cell, robot cell can be None and we can skip this check
        if robot_cell:
            robot_cell.assert_cell_state_match(robot_cell_state)

        robot_cell_state = robot_cell_state.copy()

        # TODO: Check for modified object states and change those only

        # Sanity check to ensure that rigid bodies are not attached to hidden tools
        for rigid_body_name, rigid_body_state in robot_cell_state.rigid_body_states.items():
            if rigid_body_state.attached_to_tool:
                tool_name = rigid_body_state.attached_to_tool
                if tool_name not in robot_cell_state.tool_states:
                    raise ValueError(
                        "State inconsistency: Rigid body {} is attached to a non-existent tool {}".format(
                            rigid_body_name, tool_name
                        )
                    )
                tool_state = robot_cell_state.tool_states[tool_name]
                if tool_state.is_hidden:
                    raise ValueError(
                        "State inconsistency: Rigid body {} is attached to a hidden tool {}".format(
                            rigid_body_name, tool_name
                        )
                    )

        # Update the robot configuration if it is provided
        # Note robot_cell_state.robot_configuration is a full configuration
        if robot_cell_state.robot_configuration:
            client.set_robot_configuration(robot_cell_state.robot_configuration)

        # Keep track of tool's base_frames during tool updates
        # They can be used later to update rigid body base frames that are attached to tools
        tool_base_frames = {}

        # Update the position of Tool models
        for tool_name, tool_state in robot_cell_state.tool_states.items():
            # Compute tool_base_frame for the tool
            tool_base_frame = None
            if tool_state.is_hidden:
                # There is no hide_tool method for the time being, hidden objects are simply not checked during Collision checking
                # client.hide_tool(tool_name)
                continue
            if tool_state.attached_to_group:
                # Skip if robot_configuration is not provided
                if not robot_cell_state.robot_configuration:
                    continue
                # If tool is attached to a group, update the tool's base frame using the group's FK frame
                link_name = client.robot.get_link_names(tool_state.attached_to_group)[-1]
                robot_configuration = robot_cell_state.robot_configuration
                # Get PCF of the Robot
                # planner_coordinate_frame = client.robot.model.forward_kinematics(robot_configuration, link_name)
                pcf_link_id = client.robot_link_puids[link_name]
                planner_coordinate_frame = client._get_link_frame(pcf_link_id, client.robot_puid)
                tool_base_frame = self._compute_tool_base_frame_from_planner_coordinate_frame(
                    tool_state, planner_coordinate_frame
                )
            else:
                # If the tool is not attached, update the tool's base frame using the frame in tool state
                tool_base_frame = tool_state.frame
            # Keep the tool_base_frame for later use when updating attached workpieces
            tool_base_frames[tool_name] = tool_base_frame

            # Set the frame of the tool
            client.set_tool_base_frame(tool_name, tool_base_frame)

        # Update the position of RigidBody models
        for rigid_body_name, rigid_body_state in robot_cell_state.rigid_body_states.items():
            # Compute rigid_body_base_frame for the rigid body
            rigid_body_base_frame = None
            if rigid_body_state.is_hidden:
                continue
            if rigid_body_state.attached_to_tool:
                # Skip if the tool_base_frame was not recorded in the previous step,
                # this means that the tool state is unknown due to absent robot_configuration
                if rigid_body_state.attached_to_tool not in tool_base_frames:
                    continue

                # If rigid body is attached to a tool, update the rigid body's base frame using the tool's FK frame
                tool_name = rigid_body_state.attached_to_tool
                tool_model = robot_cell.tool_models[tool_name]
                tool_state = robot_cell_state.tool_states[tool_name]

                # The following function computes the rigid body base frame from the tool base frame and attachment frame
                rigid_body_base_frame = self._compute_workpiece_frame_from_tool_base_frame(
                    rigid_body_state, tool_model, tool_base_frames[tool_name]
                )
            elif rigid_body_state.attached_to_link:
                # Skip if robot_configuration is not provided
                if not robot_cell_state.robot_configuration:
                    continue
                # If rigid body is attached to a link, update the rigid body's base frame using the link's FK frame
                link_name = rigid_body_state.attached_to_link
                robot_configuration = robot_cell_state.robot_configuration
                link_frame = client.robot.model.forward_kinematics(robot_configuration, link_name)
                attachment_frame = rigid_body_state.attachment_frame

                # Compute the RB position with its attachment frame relative to the link frame
                t_wcf_lcf = Transformation.from_frame(link_frame)
                t_lcf_rbcf = Transformation.from_frame(attachment_frame)

                t_wcf_rbcf = t_wcf_lcf * t_lcf_rbcf
                rigid_body_base_frame = Frame.from_transformation(t_wcf_rbcf)
            else:
                # If the rigid body is not attached, update the rigid body's base frame using the frame in rigid body state
                rigid_body_base_frame = rigid_body_state.frame

            # Set the frame of the rigid body
            client.set_rigid_body_base_frame(rigid_body_name, rigid_body_base_frame)

        # The client needs to keep track of the latest robot cell state
        client._robot_cell_state = robot_cell_state

        # This function updates the position of all models in the robot cell, technically we could
        # keep track of the previous state and only update the models that have changed to improve performance.
        # We can improve when we have proper profiling and performance tests.

    def set_attached_tool_and_rigid_body_state(self, state, group, planner_coordinate_frame):
        # type: (RobotCellState, str, Frame) -> None
        """Change the state of the models in the robot cell that have already been set to the Pybullet client.

        Similar to the :meth:`set_robot_cell_state` method, but affects only the tools and rigid bodies
        that are attached to the specified group.

        """

        # Housekeeping for intellisense
        planner = self  # type: PyBulletPlanner
        client = planner.client  # type: PyBulletClient
        # robot = client.robot  # type: Robot
        robot_cell = client.robot_cell  # type: RobotCell
        robot_cell_state = state.copy()  # type: RobotCellState

        tool_base_frames = {}
        for tool_name, tool_state in robot_cell_state.tool_states.items():
            # If the tool is attached to the group, update the tool's base frame using the planner_coordinate_frame
            if tool_state.attached_to_group == group:
                tool_base_frame = self._compute_tool_base_frame_from_planner_coordinate_frame(
                    tool_state, planner_coordinate_frame
                )
                tool_base_frames[tool_name] = tool_base_frame
                client.set_tool_base_frame(tool_name, tool_base_frame)

        for rigid_body_name, rigid_body_state in robot_cell_state.rigid_body_states.items():
            # Filter only the rigid bodies that are attached to tools that are processed in the previous step
            if rigid_body_state.attached_to_tool in tool_base_frames:
                tool_name = rigid_body_state.attached_to_tool
                tool_base_frame = tool_base_frames[tool_name]
                tool_model = robot_cell.tool_models[tool_name]

                # The following function computes the rigid body base frame from the tool base frame and attachment frame
                rigid_body_base_frame = self._compute_workpiece_frame_from_tool_base_frame(
                    rigid_body_state, tool_model, tool_base_frame
                )

                client.set_rigid_body_base_frame(rigid_body_name, rigid_body_base_frame)

    def _compute_workpiece_frame_from_tool_base_frame(self, rigid_body_state, tool_model, tool_base_frame):
        # type: (RigidBodyState, ToolModel, Frame) -> Frame
        """Compute the rigid body base frame from the tool base frame and attachment frame."""
        # Note: The attachment order from the World to the Workpiece are as follows:
        # t_wcf_tbcf is Tool Base Frame, describing Tool Base Coordinate Frame (TBCF) relative to World Coordinate Frame (WCF)
        t_wcf_tbcf = Transformation.from_frame(tool_base_frame)
        # t_t0cf_tcf is Tool Frame or Tool Offset, describing Tool Coordinate Frame (TCF) relative to Tool Base Frame (T0CF)
        t_tbcf_tcf = Transformation.from_frame(tool_model.frame)
        # t_tcf_ocf is workpiece's Attachment Frame, describing Object Coordinate Frame (OCF) relative to Tool Coordinate Frame (TCF)
        t_tcf_ocf = Transformation.from_frame(rigid_body_state.attachment_frame)

        # Note: The combined transformation t_wcf_ocf is the Object Coordinate Frame (OCF) relative to World Coordinate Frame (WCF)
        # It gives the position of the workpiece in the world coordinate frame
        t_wcf_ocf = t_wcf_tbcf * t_tbcf_tcf * t_tcf_ocf

        rigid_body_base_frame = Frame.from_transformation(t_wcf_ocf)
        return rigid_body_base_frame

    def _compute_tool_base_frame_from_planner_coordinate_frame(self, tool_state, planner_coordinate_frame):
        # type: (ToolState, Frame) -> Frame
        """Compute the tool base frame from the planner coordinate frame."""

        # Note: The position of the attached tool in the world coordinate frame is given by t_wcf_tbcf
        # t_wcf_t0cf is Tool0 Frame of the robot obtained from FK, describing Tool Base Frame (T0CF) relative to World Coordinate Frame (WCF)
        t_wcf_t0cf = Transformation.from_frame(planner_coordinate_frame)
        # t_t0cf_tbcf is Tool Attachment Frame, describing Tool Base Coordinate Frame (TBCF) relative to Tool0 Frame on the Robot (T0CF)
        t_t0cf_tbcf = Transformation.from_frame(tool_state.attachment_frame)

        # Combined transformation gives the position of the tool in the world coordinate frame
        t_wcf_tbcf = t_wcf_t0cf * t_t0cf_tbcf
        return Frame.from_transformation(t_wcf_tbcf)

from compas_fab.backends.interfaces import SetRobotCellState

import compas
from compas.geometry import Frame
from compas.geometry import Transformation

if not compas.IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import List  # noqa: F401
        from typing import Tuple  # noqa: F401

        from compas_fab.robots import Robot  # noqa: F401
        from compas_robots import Configuration  # noqa: F401
        from compas.geometry import Frame  # noqa: F401
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401

from compas_fab.backends.pybullet.const import STATIC_MASS


class PyBulletSetRobotCellState(SetRobotCellState):
    def _set_robot_cell_state(self, robot_cell_state):
        # type: (RobotCellState, RobotCellState) -> None
        """Change the state of the models in the robot cell that have already been set to the Pybullet client.

        This function is typically called by planning functions that accepts a start state as input.
        The user should not need to call this function directly.

        The robot cell state must match the robot cell set earlier by :meth:`set_robot_cell`.

        """
        client = self.client  # type: PyBulletClient # Trick to keep intellisense happy
        robot_cell = self.robot_cell  # type: RobotCell

        # Check if the robot cell state matches the robot cell.
        # However if the robot is the only element in the cell, robot cell can be None and we can skip this check
        if robot_cell:
            robot_cell.assert_cell_state_match(robot_cell_state)

        # TODO: Check for modified object states and change those only

        # Update the robot configuration
        # Note robot_cell_state.robot_configuration is a full configuration
        client.set_robot_configuration(robot_cell_state.robot_configuration)

        # Keep track of tool's base_frames during tool updates
        # They can be used later to update rigid body base frames that are attached to tools
        tool_base_frames = {}

        # Update the position of Tool models
        for tool_name, tool_state in robot_cell_state.tool_states.items():
            # if tool_state.is_hidden:
            #     client.hide_tool(tool_name)
            #     continue
            if tool_state.attached_to_group:
                # If tool is attached to a group, update the tool's base frame using the group's FK frame
                tool_attached_link_name = client.robot.get_link_names(tool_state.attached_to_group)[-1]
                configuration = robot_cell_state.robot_configuration
                tool_base_frame = client.robot.model.forward_kinematics(configuration, tool_attached_link_name)
            else:
                # If the tool is not attached, update the tool's base frame using the frame in tool state
                tool_base_frame = tool_state.frame
            tool_base_frames[tool_name] = tool_base_frame
            client.set_tool_base_frame(tool_name, tool_base_frame)

        # Update the position of RigidBody models
        for rigid_body_name, rigid_body_state in robot_cell_state.rigid_body_states.items():
            # if rigid_body_state.is_hidden:
            #     client.hide_rigid_body(rigid_body_name)
            #     continue
            if rigid_body_state.attached_to_tool:
                # If rigid body is attached to a tool, update the rigid body's base frame using the tool's FK frame
                tool_name = rigid_body_state.attached_to_tool
                tool_model = robot_cell.tool_models[tool_name]

                # t_tcf_ocf is Grasp, describing Object Coordinate Frame (OCF) relative to Tool Coordinate Frame (TCF)
                t_tcf_ocf = Transformation.from_frame(rigid_body_state.grasp)
                # t_t0cf_tcf is Tool Offset, describing Tool Coordinate Frame (TCF) relative to Tool Base Frame (T0CF)
                t_t0cf_tcf = Transformation.from_frame(tool_model.frame)
                # t_wcf_t0cf is Tool Base Frame, describing Tool Base Frame (T0CF) relative to World Coordinate Frame (WCF)
                t_wcf_t0cf = Transformation.from_frame(tool_base_frames[tool_name])
                # t_wcf_ocf is Object Coordinate Frame (OCF) relative to World Coordinate Frame (WCF)
                t_wcf_ocf = t_wcf_t0cf * t_t0cf_tcf * t_tcf_ocf

                rigid_body_base_frame = Frame.from_transformation(t_wcf_ocf)
            else:
                # If the rigid body is not attached, update the rigid body's base frame using the frame in rigid body state
                rigid_body_base_frame = rigid_body_state.frame
            client.set_rigid_body_base_frame(rigid_body_name, rigid_body_base_frame)

        # This function updates the position of all models in the robot cell, technically we could
        # keep track of the previous state and only update the models that have changed to improve performance.
        # We can improve when we have proper profiling and performance tests.

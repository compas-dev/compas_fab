from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas.geometry import Frame
from compas.geometry import Transformation
from compas.utilities import await_callback

from compas_fab.backends.interfaces import SetRobotCellState
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import JointState
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages.geometry_msgs import Header
from compas_fab.backends.ros.messages.geometry_msgs import Pose
from compas_fab.backends.ros.service_description import ServiceDescription

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:  # pragma: no cover
        from typing import Callable  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas_fab.backends import MoveItPlanner  # noqa: F401
        from compas_fab.backends import RosClient  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401


__all__ = [
    "MoveItSetRobotCellState",
]


class MoveItSetRobotCellState(SetRobotCellState):
    """Mix-in interface for implementing a planner's set robot cell state feature."""

    APPLY_PLANNING_SCENE = ServiceDescription(
        "/apply_planning_scene",
        "ApplyPlanningScene",
        ApplyPlanningSceneRequest,
        ApplyPlanningSceneResponse,
    )

    def set_robot_cell_state(self, robot_cell_state, options=None):
        # type: (RobotCellState, Optional[Dict]) -> None
        """Set the robot cell state to the client.

        This function is called automatically by planning functions that takes a RobotCellState as input.
        In normal use cases, you should not need to call this function directly.
        However, one exception is when you want to set the robot cell state to the client for visualization purposes.

        There are three steps to setting the robot cell state:
        1. Remove all AttachedCollisionObjects from the robot state.
        2. Update the position of all tools and rigid bodies in the scene.
        3. Re-create AttachedCollisionObjects for all attached rigid bodies and tools.

        Parameters
        ----------
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`
            The robot cell state to set.
        options : dict, optional
            Unused.

        """
        kwargs = {}
        kwargs["robot_cell_state"] = robot_cell_state
        kwargs["options"] = options or {}
        kwargs["errback_name"] = "errback"

        robot_cell = self.client.robot_cell  # type: RobotCell
        robot_cell.assert_cell_state_match(robot_cell_state)

        assert robot_cell is not None, "Robot should not be None"
        assert robot_cell_state, "Robot cell state should not be None"

        # First step is to remove all the ACO attachments, this facilitates moving the position of attached objects
        # Robot configuration is also updated next step will move objects correctly based on this new configuration
        step_1_result = await_callback(self._set_rcs_remove_aco_and_update_config_async, **kwargs)
        assert step_1_result.success, "Failed to remove attached collision objects"
        # Second step is to update the position of all tools and rigid bodies in the scene
        # Attached objects positions are updated relative to the links they attached to
        # Non-attached objects positions are relative to the robot base link (aka world)
        step_2_result = await_callback(self._set_rcs_update_co_async, **kwargs)
        assert step_2_result.success
        # Third step is to re-create the ACO attachments
        # This will ensure the objects move with the robot when planning
        step_3_result = await_callback(self._set_rcs_create_aco_async, **kwargs)
        assert step_3_result.success

        # Update the client's robot cell state
        self.client._robot_cell_state = robot_cell_state

        return (step_1_result, step_2_result, step_3_result)

    def _set_rcs_remove_aco_and_update_config_async(self, callback, errback, robot_cell_state, options):
        # type: (Callable, Callable, RobotCellState, Dict) -> None
        """Remove AttachedCollisionObject from the client.

        All AttachedCollisionObjects are removed, converting them back to CollisionObjects in the PlanningSceneWorld.
        """
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell

        # Convenience function for printing verbose messages
        def verbose_print(msg):
            if options.get("verbose", False):
                print(msg)

        # Get the last robot state
        planning_scene = planner.get_planning_scene()  # type: PlanningScene
        robot_state = planning_scene.robot_state

        # Remove all attached collision objects
        attached_collision_objects = []
        link_names = []
        for previous_aco in robot_state.attached_collision_objects:
            link_name = previous_aco.link_name
            if link_name in link_names:
                continue
            aco = AttachedCollisionObject(
                link_name=link_name,
                object=CollisionObject(
                    header=Header(frame_id=link_name),
                    id="",  # Setting id to empty string will remove all ACOs from the link
                    operation=CollisionObject.REMOVE,
                ),
                weight=1.0,
            )
            link_names.append(link_name)
            attached_collision_objects.append(aco)
            verbose_print("SET_RCS_1: Removed all ACOs from link {}".format(link_name))

        # Update robot configuration
        configuration = robot_cell_state.robot_configuration
        joint_state = JointState(
            header=Header(frame_id=robot_cell.root_name),
            name=configuration.joint_names,
            position=configuration.joint_values,
            # I'm not sure what it means when we don't set the velocity and effort
        )

        if len(attached_collision_objects) > 0:
            robot_state = RobotState(
                joint_state=joint_state,
                attached_collision_objects=attached_collision_objects,
                is_diff=True,
            )
        else:
            robot_state = RobotState(
                joint_state=joint_state,
                is_diff=True,
            )

        scene = PlanningScene(robot_state=robot_state, is_diff=True)
        request = scene.to_request(client.ros_distro)
        self.APPLY_PLANNING_SCENE(client, request, callback, errback)

    def _set_rcs_update_co_async(self, callback, errback, robot_cell_state, options):
        # type: (Callable, Callable, RobotCellState, Dict) -> None
        """Update the position of CollisionObjects in the client.

        Non-attached objects (RigidBody and Tools) are updated in the PlanningSceneWorld.

        This function is called automatically by planning functions that takes a RobotCellState as input.

        """
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell
        root_name = robot_cell.root_name

        # Convenience function for printing verbose messages
        def verbose_print(msg):
            if options.get("verbose", False):
                print(msg)

        # Update pose of all non-attached rigid bodies
        collision_objects = []
        for rb_id, rigid_body_state in robot_cell_state.rigid_body_states.items():
            # For rigid bodies attached to links
            if rigid_body_state.attached_to_link:
                collision_object = CollisionObject(
                    header=Header(frame_id=rigid_body_state.attached_to_link),
                    id=rb_id,
                    pose=Pose.from_frame(rigid_body_state.attachment_frame or Frame.worldXY()),
                    operation=CollisionObject.MOVE,
                )
                collision_objects.append(collision_object)
                verbose_print(
                    "SET_RCS_2: Moved CO {} for attachment to position of link {}".format(
                        rb_id, rigid_body_state.attached_to_link
                    )
                )
            # For rigid bodies attached to tools
            elif rigid_body_state.attached_to_tool:
                t_pcf_ocf = client.robot_cell.t_pcf_ocf(robot_cell_state, rb_id)
                tool_parent_group = robot_cell_state.tool_states[rigid_body_state.attached_to_tool].attached_to_group
                end_effector_link_name = robot_cell.get_end_effector_link_name(group=tool_parent_group)
                collision_object = CollisionObject(
                    header=Header(frame_id=end_effector_link_name),
                    id=rb_id,
                    pose=Pose.from_transformation(t_pcf_ocf),
                    operation=CollisionObject.MOVE,
                )
                collision_objects.append(collision_object)
                verbose_print(
                    "SET_RCS_2: Moved CO {} for attachment to position of tool {}".format(
                        rb_id, rigid_body_state.attached_to_tool
                    )
                )
            # For stationary rigid bodies
            else:
                collision_object = CollisionObject(
                    header=Header(frame_id=root_name),
                    id=rb_id,
                    pose=Pose.from_frame(rigid_body_state.frame or Frame.worldXY()),
                    operation=CollisionObject.MOVE,
                )
                collision_objects.append(collision_object)
                verbose_print("SET_RCS_2: Moved CO {} to frame {}".format(rb_id, rigid_body_state.frame))

        # Update the tools
        for tool_id, tool_state in robot_cell_state.tool_states.items():
            tool_model = client.robot_cell.tool_models[tool_id]
            # For tools attached to groups
            if tool_state.attached_to_group:
                assert tool_state.attachment_frame, "Tool state attachment frame should not be None for attached tools."
                end_effector_link_name = robot_cell.get_end_effector_link_name(group=tool_state.attached_to_group)

                # Update the base link of the tool
                base_link_name = tool_model.root.name
                collision_object_id = "{}_{}".format(tool_id, base_link_name)
                collision_object = CollisionObject(
                    header=Header(frame_id=end_effector_link_name),
                    id=collision_object_id,
                    pose=Pose.from_frame(tool_state.attachment_frame),
                    operation=CollisionObject.MOVE,
                )
                collision_objects.append(collision_object)
                verbose_print(
                    "SET_RCS_2: Moved Tool Base Link {} to attachment at Robot Link {}".format(
                        collision_object_id, end_effector_link_name
                    )
                )

                # If the tool is a kinematic tool, it has configurable joints
                # We need to find the poses of each links
                if len(tool_model.get_configurable_joints()) > 0:
                    tool_configuration = tool_state.configuration or tool_model.zero_configuration()
                    transformed_joint_frames = tool_model.transformed_frames(tool_configuration)
                    t_pcf_t0cf = Transformation.from_frame(tool_state.attachment_frame)
                    for joint_frame, joint in zip(transformed_joint_frames, list(tool_model.iter_joints())):
                        t_t0cf_lcf = Transformation.from_frame(joint_frame)
                        child_link = tool_model.get_joint_by_name(joint.name).child_link
                        collision_object_id = "{}_{}".format(tool_id, child_link.name)
                        t_pcf_lcf = t_pcf_t0cf * t_t0cf_lcf
                        collision_object = CollisionObject(
                            header=Header(frame_id=end_effector_link_name),
                            id=collision_object_id,
                            pose=Pose.from_transformation(t_pcf_lcf),
                            operation=CollisionObject.MOVE,
                        )
                        collision_objects.append(collision_object)
                        verbose_print(
                            "SET_RCS_2: Moved Tool Link {} to attachment at Robot Link {}".format(
                                collision_object_id, end_effector_link_name
                            )
                        )

            # For stationary tools
            else:
                assert tool_state.frame, "Tool state frame should not be None for detached tools."
                # Update the base link of the tool
                base_link_name = tool_model.root.name
                collision_object_id = "{}_{}".format(tool_id, base_link_name)
                collision_object = CollisionObject(
                    header=Header(frame_id=root_name),
                    id=collision_object_id,
                    pose=Pose.from_frame(tool_state.frame),
                    operation=CollisionObject.MOVE,
                )
                collision_objects.append(collision_object)
                verbose_print("SET_RCS_2: Moved Tool Base Link {} to stationary pose".format(collision_object_id))

                # If the tool is a kinematic tool, it has configurable joints
                # We need to find the poses of each links
                if len(tool_model.get_configurable_joints()) > 0:
                    tool_configuration = tool_state.configuration or tool_model.zero_configuration()
                    transformed_joint_frames = tool_model.transformed_frames(tool_configuration)
                    t_wcf_t0cf = Transformation.from_frame(tool_state.frame)
                    for joint_frame, joint in zip(transformed_joint_frames, list(tool_model.iter_joints())):
                        t_t0cf_lcf = Transformation.from_frame(joint_frame)
                        child_link = tool_model.get_joint_by_name(joint.name).child_link
                        collision_object_id = "{}_{}".format(tool_id, child_link.name)
                        t_wcf_lcf = t_wcf_t0cf * t_t0cf_lcf
                        collision_object = CollisionObject(
                            header=Header(frame_id=root_name),
                            id=collision_object_id,
                            pose=Pose.from_transformation(t_wcf_lcf),
                            operation=CollisionObject.MOVE,
                        )
                        collision_objects.append(collision_object)
                        verbose_print("SET_RCS_2: Moved Tool Link {} to stationary pose".format(collision_object_id))

        # Apply the planning scene to the client
        world = PlanningSceneWorld(collision_objects=collision_objects)
        scene = PlanningScene(world=world, is_diff=True)
        request = scene.to_request(client.ros_distro)
        self.APPLY_PLANNING_SCENE(client, request, callback, errback)

    def _set_rcs_create_aco_async(self, callback, errback, robot_cell_state, options):
        # type: (Callable, Callable, RobotCellState, Dict) -> None
        """Update the position of CollisionObjects in the client.

        Non-attached objects (RigidBody and Tools) are updated in the PlanningSceneWorld.

        This function is called automatically by planning functions that takes a RobotCellState as input.

        """
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = planner.client  # type: RosClient
        robot_cell = client.robot_cell

        # Convenience function for printing verbose messages
        def verbose_print(msg):
            if options.get("verbose", False):
                print(msg)

        # ACO for attached rigid bodies
        attached_collision_objects = []
        for rb_id, rigid_body_state in robot_cell_state.rigid_body_states.items():
            # For rigid bodies attached to links
            if rigid_body_state.attached_to_link:
                link_name = rigid_body_state.attached_to_link
                aco = AttachedCollisionObject(
                    link_name=link_name,
                    object=CollisionObject(
                        header=Header(frame_id=link_name),
                        id=rb_id,
                        operation=CollisionObject.ADD,
                    ),
                    touch_links=rigid_body_state.touch_links,
                    weight=1.0,
                )
                attached_collision_objects.append(aco)
                verbose_print("SET_RCS_3: Added ACO for RigidBody {} to link {}".format(rb_id, link_name))
            # For rigid bodies attached to tools
            elif rigid_body_state.attached_to_tool:
                tool_parent_group = robot_cell_state.tool_states[rigid_body_state.attached_to_tool].attached_to_group
                link_name = robot_cell.get_end_effector_link_name(group=tool_parent_group)
                aco = AttachedCollisionObject(
                    link_name=link_name,
                    object=CollisionObject(
                        header=Header(frame_id=link_name),
                        id=rb_id,
                        operation=CollisionObject.ADD,
                    ),
                    touch_links=rigid_body_state.touch_links,
                    weight=1.0,
                )
                attached_collision_objects.append(aco)
                verbose_print(
                    "SET_RCS_3: Added ACO for RigidBody {} to tool {}".format(rb_id, rigid_body_state.attached_to_tool)
                )
        # ACO for attached tools
        for tool_id, tool_state in robot_cell_state.tool_states.items():
            tool_model = client.robot_cell.tool_models[tool_id]
            if tool_state.attached_to_group:
                end_effector_link_name = robot_cell.get_end_effector_link_name(group=tool_state.attached_to_group)
                for link in tool_model.iter_links():
                    collision_object_id = "{}_{}".format(tool_id, link.name)

                    aco = AttachedCollisionObject(
                        link_name=end_effector_link_name,
                        object=CollisionObject(
                            header=Header(frame_id=end_effector_link_name),
                            id=collision_object_id,
                            operation=CollisionObject.ADD,
                        ),
                        touch_links=tool_state.touch_links,
                        weight=1.0,
                    )
                    attached_collision_objects.append(aco)
                    verbose_print(
                        "SET_RCS_3: Added ACO for Tool Link {} to Robot Link {}".format(
                            collision_object_id, end_effector_link_name
                        )
                    )

        # Apply the planning scene to the client
        robot_state = RobotState(
            attached_collision_objects=attached_collision_objects,
            is_diff=True,
        )
        scene = PlanningScene(robot_state=robot_state, is_diff=True)

        request = scene.to_request(client.ros_distro)
        self.APPLY_PLANNING_SCENE(client, request, callback, errback)

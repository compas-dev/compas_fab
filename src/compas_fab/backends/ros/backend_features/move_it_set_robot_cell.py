from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas.utilities import await_callback

from compas_fab.backends.interfaces import SetRobotCell
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.messages.geometry_msgs import Header
from compas_fab.backends.ros.messages.geometry_msgs import Pose
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages.shape_msgs import Mesh
from compas_fab.backends.ros.service_description import ServiceDescription

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Callable  # noqa: F401
        from typing import Dict  # noqa: F401
        from typing import Optional  # noqa: F401

        from compas_fab.backends import MoveItPlanner  # noqa: F401
        from compas_fab.backends import PyBulletClient  # noqa: F401
        from compas_fab.robots import RobotCell  # noqa: F401
        from compas_fab.robots import RobotCellState  # noqa: F401

__all__ = [
    "MoveItSetRobotCell",
]


class MoveItSetRobotCell(SetRobotCell):
    """Callable to add a collision mesh to the planning scene."""

    APPLY_PLANNING_SCENE = ServiceDescription(
        "/apply_planning_scene",
        "ApplyPlanningScene",
        ApplyPlanningSceneRequest,
        ApplyPlanningSceneResponse,
    )
    ATTACHED_OBJECTS_WEIGHT = 1.0

    def set_robot_cell(self, robot_cell, robot_cell_state=None, options=None):
        # type: (RobotCell, Optional[RobotCellState], Optional[Dict]) -> None
        """Pass the models in the robot cell to the planning client.

        The client keeps the robot cell models in memory and uses them for planning.
        Calling this method will override the previous robot cell in the client.
        It should be called only if the robot cell models have changed.

        """

        kwargs = {}
        kwargs["robot_cell"] = robot_cell
        kwargs["robot_cell_state"] = robot_cell_state
        kwargs["options"] = options
        kwargs["errback_name"] = "errback"

        return await_callback(self.set_robot_cell_async, **kwargs)

    def set_robot_cell_async(self, callback, errback, robot_cell, robot_cell_state=None, options=None):
        # type: (Callable, Callable, RobotCell, Optional[RobotCellState], Optional[Dict]) -> None
        """Pass the models in the robot cell to the planning client.

        The client keeps the robot cell models in memory and uses them for planning.
        Calling this method will override the previous robot cell in the client.
        It should be called only if the robot cell models have changed.

        """
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = self.client  # type: PyBulletClient
        robot = client.robot

        assert robot_cell, "Robot cell should not be None"
        assert robot_cell.robot, "Robot cell should have a robot"

        # First implementation test
        # We remove all objects and add them again, the diff is left to the client

        # NOTE: Diff are all false for now. In the future we can generate actual diff here
        # to reduce already-existing geometry being sent to over ROS

        # Add the rigid bodies to the planning scene
        collision_objects = {}
        for rigid_body_id, rigid_body in robot_cell.rigid_body_models.items():

            kwargs = {}
            # The frame_id needs be the root name of the robot
            kwargs["header"] = Header(frame_id=robot.root_name)
            # id is the identifier of the rigid body
            kwargs["id"] = rigid_body_id
            # List of `compas_fab.backends.ros.messages.shape_msgs.Meshes`
            kwargs["meshes"] = [Mesh.from_mesh(m) for m in rigid_body.collision_meshes]
            # List of `compas_fab.backends.ros.messages.geometry_msgs.Pose`
            kwargs["mesh_poses"] = [Pose() for _ in rigid_body.collision_meshes]
            kwargs["operation"] = CollisionObject.ADD
            if robot_cell_state:
                rigid_body_frame = robot_cell_state.rigid_body_states[rigid_body_id].frame
                kwargs["pose"] = Pose.from_frame(rigid_body_frame)
            else:
                kwargs["pose"] = Pose()

            co = CollisionObject(**kwargs)
            collision_objects[rigid_body_id] = co

        # Add attached collision objects to the robot_state
        attached_collision_objects = {}
        if robot_cell_state:
            for rigid_body_id in robot_cell_state.get_attached_rigid_body_ids():
                rigid_body_state = robot_cell_state.rigid_body_states[rigid_body_id]
                collision_object = collision_objects[rigid_body_id]

                kwargs = {}
                kwargs["link_name"] = rigid_body_state.attached_to_link
                kwargs["object"] = collision_object
                kwargs["touch_links"] = rigid_body_state.touch_links
                kwargs["weight"] = self.ATTACHED_OBJECTS_WEIGHT

                collision_object.pose = Pose.from_frame(rigid_body_state.attachment_frame)
                attached_collision_objects[rigid_body_id] = AttachedCollisionObject(**kwargs)
        robot_state = RobotState(attached_collision_objects=list(attached_collision_objects.values()), is_diff=False)

        world = PlanningSceneWorld(collision_objects=list(collision_objects.values()))
        scene = PlanningScene(robot_state=robot_state, world=world, is_diff=False)

        request = scene.to_request(client.ros_distro)
        self.APPLY_PLANNING_SCENE(client, request, callback, errback)

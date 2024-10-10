from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas.utilities import await_callback

from compas_fab.backends.interfaces import SetRobotCell
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import CollisionMesh
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.messages.geometry_msgs import Header
from compas_fab.backends.ros.messages.geometry_msgs import Pose
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages.shape_msgs import Mesh
from compas_fab.backends.ros.service_description import ServiceDescription
from compas_robots.model import LinkGeometry

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

        MoveIt keeps all the robot cell models in memory and uses them for planning.

        Calling this method will override the previous robot cell passed to the client.
        Setting the robot cell is a slow operation, as it involves sending the
        geometry of the robot cell to the planning client.
        It should be called only if the robot cell models have changed.

        This function is responsible for creating moveit_msgs/CollisionObject messages
        for each RigidBody in the robot cell with the ADD operation.

        Tools are also considered as MoveIt CollisionsObjects in the same way.
        Because MoveIt backend does not support kinematic tools (with movable parts),
        they are treated as static objects fixed at their zero configuration.
        """
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = planner.client  # type: PyBulletClient
        robot = client.robot

        assert robot_cell, "Robot cell should not be None"
        assert robot_cell.robot, "Robot cell should have a robot"

        last_planning_scene_world = planner._last_planning_scene_world

        # Retrieve the previous planning scene to keep the allowed collision matrix
        # previous_planning_scene = planner.get_planning_scene()  # type: PlanningScene
        # print(previous_planning_scene.allowed_collision_matrix)

        # `name` == '(noname)+'
        # `object_colors`, `fixed_frame_transformation`, `link_padding` and `link_scale` are empty == []
        # `robot` == [{'enabled': [True]}]

        # NOTE: This function will only create CollisionObjects (not attached),
        # The `set_robot_cell_state`` is responsible of the creation of AttachedCollisionObjects.

        collision_objects = []

        # Remove rigid bodies from the previous planning scene that are not in the new robot cell
        for rigid_body_id in list(planner._current_rigid_body_hashes):
            if rigid_body_id not in robot_cell.rigid_body_models:
                co = CollisionObject(
                    id=rigid_body_id,
                    operation=CollisionObject.REMOVE,
                )
                collision_objects.append(co)
                # Remove the hash from the dictionary
                del planner._current_rigid_body_hashes[rigid_body_id]
                print("Rigid body {} removed from the planning scene".format(rigid_body_id))

        # Add rigid bodies to the planning scene
        # Skip those that are already in the planning scene

        for rigid_body_id, rigid_body in robot_cell.rigid_body_models.items():
            rigid_body_hash = rigid_body.sha256()
            # Compare the hash of the rigid body with the previous one
            if rigid_body_id in planner._current_rigid_body_hashes:
                if rigid_body_hash == planner._current_rigid_body_hashes[rigid_body_id]:
                    print("Rigid body {} skipped because is already in the planning scene".format(rigid_body_id))
                    continue

            # Create new CollisionObject to be passed to backend
            ros_meshes = [Mesh.from_mesh(m) for m in rigid_body.collision_meshes]
            collision_object = CollisionObject(
                # Header is robot.root_name when the object is not attached
                header=Header(frame_id=robot.root_name),
                # The identifier of the rigid body, matches with the robot_cell rigid_body_id
                id=rigid_body_id,
                # List of `compas_fab.backends.ros.messages.shape_msgs.Meshes`
                meshes=ros_meshes,
                # List of Pose, one for each mesh
                # NOTE: We leave the `mesh_poses` at origin because it cannot be moved later
                mesh_poses=[Pose() for _ in ros_meshes],
                # `pose` can be modified later, at the moment it is not set.
                pose=Pose(),
                # Operation is always ADD
                operation=CollisionObject.ADD,
            )
            collision_objects.append(collision_object)
            print("Rigid body {} added to the planning scene".format(rigid_body_id))

            # Update the hash of the rigid body
            planner._current_rigid_body_hashes[rigid_body_id] = rigid_body_hash

        # # Add tools to the planning scene
        # for tool_id, tool_model in robot_cell.tool_models.items():
        #     # For each tool, iterate through its links, each link that has geometry(s) will create one collision object
        #     # They are named as "[tool_name]_[link_name]_collision"
        #     # The naming convention here is important, as we are planning to love the links later for a kinematic tool.
        #     for link in tool_model.iter_links():
        #         collision_meshes = []
        #         # NOTE: There can be multiple Collision objects in a link, we flatten them to
        #         for collision in link.collision:
        #             collision_meshes += list(LinkGeometry._get_item_meshes(collision))
        #         # For each mesh in the link, create a CollisionMesh
        #         kwargs = {}
        #         # The frame_id needs be the root name of the robot
        #         kwargs["header"] = Header(frame_id=robot.root_name)
        #         # id is the identifier of the rigid body
        #         # In theory the `tool_id` from the tool_model dictionary is the same as the `tool_model.name`.
        #         tool_collision_mesh_name = "{}_{}_collision".format(tool_id, link.name)
        #         kwargs["id"] = tool_collision_mesh_name
        #         # List of `compas_fab.backends.ros.messages.shape_msgs.Meshes`
        #         kwargs["meshes"] = [Mesh.from_mesh(m) for m in collision_meshes]
        #         # List of `compas_fab.backends.ros.messages.geometry_msgs.Pose`, individual pose for the links
        #         # Pose will be set by set_robot_cell_state later
        #         kwargs["mesh_poses"] = [Pose() for _ in collision_meshes]
        #         kwargs["operation"] = CollisionObject.ADD
        #         # Attachment pose for the tool to the attached_link
        #         # Pose will be set by set_robot_cell_state later
        #         kwargs["pose"] = Pose()
        #         co = CollisionObject(**kwargs)
        #         collision_objects[tool_collision_mesh_name] = co

        world = PlanningSceneWorld(collision_objects=collision_objects)
        # NOTE: There is no need to update or change the Allowed Collision Matrix here
        # By setting `is_diff` to True, the allowed collision matrix for the robot will
        # not be overridden.
        scene = PlanningScene(world=world, is_diff=True)

        request = scene.to_request(client.ros_distro)
        self.APPLY_PLANNING_SCENE(client, request, callback, errback)


# The allowed_collision_matrix is a dictionary with a structure similar to the following:
# {
#     "entry_names": [
#         "base_link_inertia",
#         "forearm_link",
#         "shoulder_link",
#         "upper_arm_link",
#         "wrist_1_link",
#         "wrist_2_link",
#         "wrist_3_link",
#     ],
#     "entry_values": [
#         {"enabled": [False, False, True, True, True, False, False]},
#         {"enabled": [False, False, False, True, True, False, False]},
#         {"enabled": [True, False, False, True, True, True, False]},
#         {"enabled": [True, True, True, False, False, False, False]},
#         {"enabled": [True, True, True, False, False, True, True]},
#         {"enabled": [False, False, True, False, True, False, True]},
#         {"enabled": [False, False, False, False, True, True, False]},
#     ],
#     "default_entry_names": [],
#     "default_entry_values": [],
# }

# The robot_state is a RobotState object with a structure similar to the following:
# {
#     "joint_state": {
#         "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": "base_link"},
#         "name": [
#             "shoulder_pan_joint",
#             "shoulder_lift_joint",
#             "elbow_joint",
#             "wrist_1_joint",
#             "wrist_2_joint",
#             "wrist_3_joint",
#         ],
#         "position": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#         "velocity": [],
#         "effort": [],
#     },
#     "multi_dof_joint_state": {
#         "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": "base_link"},
#         "joint_names": [],
#         "transforms": [],
#         "twist": [],
#         "wrench": [],
#     },
#     "attached_collision_objects": [],
#     "is_diff": False,
# }

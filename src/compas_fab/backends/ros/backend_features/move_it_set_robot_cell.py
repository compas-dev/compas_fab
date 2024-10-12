from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY
from compas.utilities import await_callback
from compas_robots.model import LinkGeometry

from compas_fab.backends.interfaces import SetRobotCell
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.messages import RobotState
from compas_fab.backends.ros.messages.geometry_msgs import Header
from compas_fab.backends.ros.messages.geometry_msgs import Pose
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
        from compas_fab.backends import RosClient  # noqa: F401
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
        This function will perform a comparison between the previous robot cell and not modify the
        objects that are identical and already in the planning scene.
        Note that the comparison can still be slow,
        therefore it should be called only if the robot cell models have changed.

        Note that after modifying the robot cell, the robot_cell_state in the backend is undetermined.
        However, the next planning request will call `set_robot_cell_state` automatically to update
        the robot cell state.

        Parameters
        ----------
        robot_cell : :class:`compas_fab.robots.RobotCell`
            The robot cell instance to set on the client.
        robot_cell_state : :class:`compas_fab.robots.RobotCellState`, optional
            The robot cell state instance to set on the client after setting the robot cell.
        options : dict, optional
            Unused.
        """
        planner = self  # type: MoveItPlanner  # noqa: F841

        kwargs = {}
        kwargs["options"] = options
        kwargs["errback_name"] = "errback"

        # The first step is to remove all the ACO because we cannot remove a CO if it is attached
        step_1_result = await_callback(self._remove_aco_async, **kwargs)
        assert step_1_result.success, "Failed to remove attached collision objects"

        # Second step is to remove the non-existent objects and add the new ones
        # Objects that has the same id and the same object hash will be skipped
        # Objects that has the same id but different object hash will be updated
        kwargs["robot_cell"] = robot_cell
        step_2_result = await_callback(self._modify_co_async, **kwargs)
        assert step_2_result.success, "Failed to modify the objects in the robot cell"

        # Update the robot cell in the client
        self.client._robot_cell = robot_cell

        # If the robot cell state is provided, update it in the client
        if robot_cell_state:
            planner.set_robot_cell_state(robot_cell_state)

        return (step_1_result, step_2_result)

    def _remove_aco_async(self, callback, errback, options=None):
        # type: (Callable, Callable, RobotCellState, Optional[Dict]) -> None
        """Remove AttachedCollisionObject from the client.

        All AttachedCollisionObjects are removed,
        converting them back to CollisionObjects in the PlanningSceneWorld.

        This is necessary because Attached CollisionObjects cannot be removed.
        """
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = planner.client  # type: RosClient

        # Get the last robot state
        planning_scene = planner.get_planning_scene()  # type: PlanningScene
        robot_state = planning_scene.robot_state

        # Remove all attached collision objects
        attached_collision_objects = []
        link_names = []  # Keep track of the links that have been pruned
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
            print("SET_RC_1: Removed all ACOs from link {}".format(link_name))

        # NOTE: I'm not sure how we can skip the request if there are no ACOs to remove
        robot_state = RobotState(
            attached_collision_objects=attached_collision_objects,
            is_diff=True,
        )

        scene = PlanningScene(robot_state=robot_state, is_diff=True)
        request = scene.to_request(client.ros_distro)
        self.APPLY_PLANNING_SCENE(client, request, callback, errback)

    def _modify_co_async(self, callback, errback, robot_cell, options=None):
        # type: (Callable, Callable, RobotCell, Optional[Dict]) -> None
        """

        This function is responsible for creating moveit_msgs/CollisionObject messages
        for each RigidBody in the robot cell with the ADD operation.

        Tools are also considered as CollisionsObject in the same way.
        Because MoveIt does not support kinematic tools (with movable parts) natively,
        each link is exported as a separate CollisionObject so that they can be attached differently later.

        """
        planner = self  # type: MoveItPlanner  # noqa: F841
        client = planner.client  # type: PyBulletClient
        robot = client.robot

        assert robot_cell, "Robot cell should not be None"
        assert robot_cell.robot, "Robot cell should have a robot"

        # NOTE: This function will only create CollisionObjects (not attached),
        # The `set_robot_cell_state`` is responsible of the creation of AttachedCollisionObjects.

        collision_objects = []

        # Step 0 filter Rigid Bodies that has no collision geometry
        rigid_body_models = {k: v for k, v in robot_cell.rigid_body_models.items() if v.collision_meshes}

        # Step 1
        # Remove rigid bodies from the previous planning scene that are not in the new robot cell
        for rigid_body_id in list(planner._current_rigid_body_hashes):
            if rigid_body_id not in rigid_body_models:
                co = CollisionObject(
                    id=rigid_body_id,
                    operation=CollisionObject.REMOVE,
                )
                collision_objects.append(co)
                # Remove the hash from the dictionary
                del planner._current_rigid_body_hashes[rigid_body_id]
                print("SET_RC_2: Rigid body '{}' removed from the planning scene".format(rigid_body_id))

        # Step 2
        # Add / update rigid bodies to the planning scene
        # Skip those that are already in the planning scene

        for rigid_body_id, rigid_body in rigid_body_models.items():
            rigid_body_hash = rigid_body.sha256()
            # Compare the hash of the rigid body with the previous one
            if rigid_body_id in planner._current_rigid_body_hashes:
                if rigid_body_hash == planner._current_rigid_body_hashes[rigid_body_id]:
                    print(
                        "SET_RC_2: Rigid body '{}' skipped because is already in the planning scene".format(
                            rigid_body_id
                        )
                    )
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
            print("SET_RC_2: Rigid body '{}' added to the planning scene".format(rigid_body_id))

            # Update the hash of the rigid body
            planner._current_rigid_body_hashes[rigid_body_id] = rigid_body_hash

        # Step 3
        # Tools are also considered as CollisionObjects in the planning scene
        # Tool CollisionObjects have an id equal to `tool_id` + "_" + `link_name`

        tool_ids = []
        for tool_id, tool_model in robot_cell.tool_models.items():
            for link in tool_model.iter_links():
                if link.collision:
                    tool_ids.append("{}_{}".format(tool_id, link.name))

        # Remove previous tools that are not in the new robot cell
        for collision_object_id in list(planner._current_tool_hashes):
            if collision_object_id not in tool_ids:
                co = CollisionObject(
                    id=collision_object_id,
                    operation=CollisionObject.REMOVE,
                )
                collision_objects.append(co)
                # Remove the hash from the dictionary
                del planner._current_tool_hashes[collision_object_id]
                print("SET_RC_2: Tool Link '{}' removed from the planning scene".format(collision_object_id))

        # Step 4
        # Add / updated tools to the planning scene
        for tool_id, tool_model in robot_cell.tool_models.items():
            # For each tool, iterate through its links, each link that has geometry(s) will create one collision object
            for link in tool_model.iter_links():
                # In theory the `tool_id` from the tool_model dictionary is the same as the `tool_model.name`.
                collision_object_id = "{}_{}".format(tool_id, link.name)
                link_hash = link.sha256()

                # Skip links that have no collision geometry
                if not link.collision:
                    print(
                        "SET_RC_2: Link '{}' skipped because it has no collision geometry".format(collision_object_id)
                    )
                    continue
                # Skip links that are already in the backend
                if collision_object_id in planner._current_tool_hashes:
                    if link_hash == planner._current_tool_hashes[collision_object_id]:
                        print(
                            "SET_RC_2: Link '{}' skipped because it is already in the planning scene".format(
                                collision_object_id
                            )
                        )
                        continue

                # Construct a new CollisionObject for the link
                collision_meshes = []
                # NOTE: There can be multiple Collision objects in a link, we put them all into the same CollisionObject
                for collision in link.collision:
                    collision_meshes += list(LinkGeometry._get_item_meshes(collision))
                # For each mesh in the link, create a CollisionMesh
                kwargs = {}
                # The frame_id needs be the root name of the robot
                kwargs["header"] = Header(frame_id=robot.root_name)
                # id is the identifier of the rigid body
                kwargs["id"] = collision_object_id
                # List of `compas_fab.backends.ros.messages.shape_msgs.Meshes`
                kwargs["meshes"] = [Mesh.from_mesh(m) for m in collision_meshes]
                # List of `compas_fab.backends.ros.messages.geometry_msgs.Pose`, individual pose for the links
                # Pose will be set by set_robot_cell_state later
                kwargs["mesh_poses"] = [Pose() for _ in collision_meshes]
                kwargs["operation"] = CollisionObject.ADD
                # Attachment pose for the tool to the attached_link
                # Pose will be set by set_robot_cell_state later
                kwargs["pose"] = Pose()
                co = CollisionObject(**kwargs)
                collision_objects.append(co)
                print("SET_RC_2: Tool Link '{}' added to the planning scene".format(link.name))

                # Update the hash of the tool
                planner._current_tool_hashes[collision_object_id] = link_hash

        world = PlanningSceneWorld(collision_objects=collision_objects)
        # NOTE: There is no need to update or change the Allowed Collision Matrix here
        # By setting `is_diff` to True, the allowed collision matrix for the robot will
        # not be overridden.
        scene = PlanningScene(world=world, is_diff=True)

        request = scene.to_request(client.ros_distro)
        self.APPLY_PLANNING_SCENE(client, request, callback, errback)


# Below is an example for the allowed_collision_matrix object received from ROS:
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

# Below is an example for the RobotState object received from ROS:
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

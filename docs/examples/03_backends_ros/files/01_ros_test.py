from compas.datastructures import Mesh
from compas.geometry import Frame

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RigidBody
from compas_fab.robots import RobotCellState

with RosClient() as client:
    robot = client.load_robot()
    planner = MoveItPlanner(client)

    # =========
    # Example 1
    # =========

    # floor_mesh = Mesh.from_stl(compas_fab.get("planning_scene/floor.stl"))
    # robot_cell.rigid_body_models["floor"] = RigidBody.from_mesh(floor_mesh)

    planner.reset_planning_scene()
    import time

    time.sleep(0.5)

    cone_mesh = Mesh.from_stl(compas_fab.get("planning_scene/cone.stl")).scaled(1.1)

    link_name = "tool0"
    touch_links = ["flange", "tool0", "wrist_3_link"]
    rigid_body_name = "cone"
    ground_frame = Frame([1, 1, 0], [1, 0, 0], [0, 1, 0])
    header_frame_id = robot.root_name
    attachment_frame = Frame.worldXY()

    # --------------------------------------------------------------

    from compas_fab.backends.ros.messages import AttachedCollisionObject
    from compas_fab.backends.ros.messages import CollisionObject
    from compas_fab.backends.ros.messages import RobotState
    from compas_fab.backends.ros.messages import PlanningScene
    from compas_fab.backends.ros.messages import PlanningSceneWorld
    from compas_fab.backends.ros.messages import Pose
    from compas_fab.backends.ros.messages import Header
    from compas_fab.backends.ros.messages import Mesh as ROSMesh

    from compas.utilities import await_callback

    def _async_add_co(callback, errback):
        collision_object = CollisionObject(
            header=Header(frame_id=robot.model.root.name),
            id=rigid_body_name,
            meshes=[ROSMesh.from_mesh(cone_mesh)],
            mesh_poses=[Pose()],
            pose=Pose.from_frame(ground_frame),
            operation=CollisionObject.ADD,
        )
        world = PlanningSceneWorld(collision_objects=[collision_object])
        scene = PlanningScene(world=world, is_diff=True)
        request = scene.to_request(client.ros_distro)
        planner.APPLY_PLANNING_SCENE(client, request, callback, errback)

    kwargs = {}
    kwargs["errback_name"] = "errback"
    print(await_callback(_async_add_co, **kwargs))

    input("Press Enter to continue...")

    # NOTE: We need to (1) move the CO to the right location and (2) create the SCO.
    #       We cannot do both in the same request, so there are two async calls
    #       It is also not possible to move the CO if it is already made to ACO
    def _async_move_co_to_attach_location(callback, errback):

        collision_object = CollisionObject(
            header=Header(frame_id=link_name),
            id=rigid_body_name,
            pose=Pose.from_frame(attachment_frame),
            operation=CollisionObject.MOVE,
        )
        world = PlanningSceneWorld(collision_objects=[collision_object])
        scene = PlanningScene(world=world, is_diff=True)
        request = scene.to_request(client.ros_distro)
        planner.APPLY_PLANNING_SCENE(client, request, callback, errback)

    print(await_callback(_async_move_co_to_attach_location, **kwargs))
    input("Press Enter to continue...")

    def _async_attach_co(callback, errback):
        aco = AttachedCollisionObject(
            link_name=link_name,
            object=CollisionObject(
                header=Header(frame_id=link_name),
                id=rigid_body_name,
                operation=CollisionObject.ADD,
            ),
            touch_links=touch_links,
            weight=1.0,
        )

        robot_state = RobotState(attached_collision_objects=[aco], is_diff=True)
        scene = PlanningScene(robot_state=robot_state, is_diff=True)
        request = scene.to_request(client.ros_distro)
        planner.APPLY_PLANNING_SCENE(client, request, callback, errback)

    kwargs = {}
    kwargs["errback_name"] = "errback"
    print(await_callback(_async_attach_co, **kwargs))
    input("Press Enter to continue...")

    def _async_detach(callback, errback):
        # To detach ACM
        aco = AttachedCollisionObject(
            link_name=link_name,
            object=CollisionObject(
                header=Header(frame_id=link_name),
                id=rigid_body_name,
                operation=CollisionObject.REMOVE,
            ),
            # weight=1.0,
        )
        robot_state = RobotState(attached_collision_objects=[aco], is_diff=True)

        # Sets the position of stationary rigid bodies
        co = CollisionObject(
            header=Header(frame_id=robot.model.root.name),
            id=rigid_body_name,
            pose=Pose.from_frame(ground_frame),
            operation=CollisionObject.MOVE,
        )

        planning_scene_world = PlanningSceneWorld(collision_objects=[co])

        scene = PlanningScene(robot_state=robot_state, world=planning_scene_world, is_diff=True)
        request = scene.to_request(client.ros_distro)
        planner.APPLY_PLANNING_SCENE(client, request, callback, errback)

    print(await_callback(_async_detach, **kwargs))

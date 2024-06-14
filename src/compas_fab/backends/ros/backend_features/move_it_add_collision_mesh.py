from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import AddCollisionMesh
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    "MoveItAddCollisionMesh",
]


class MoveItAddCollisionMesh(AddCollisionMesh):
    """Callable to add a collision mesh to the planning scene."""

    APPLY_PLANNING_SCENE = ServiceDescription(
        "/apply_planning_scene",
        "ApplyPlanningScene",
        ApplyPlanningSceneRequest,
        ApplyPlanningSceneResponse,
    )

    def add_collision_mesh(self, collision_mesh, options=None):
        """Add a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            Object containing the collision mesh to be added.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        kwargs = {}
        kwargs["collision_mesh"] = collision_mesh
        kwargs["errback_name"] = "errback"

        return await_callback(self.add_collision_mesh_async, **kwargs)

    def add_collision_mesh_async(self, callback, errback, collision_mesh):
        co = CollisionObject.from_collision_mesh(collision_mesh)
        co.operation = CollisionObject.ADD
        world = PlanningSceneWorld(collision_objects=[co])
        scene = PlanningScene(world=world, is_diff=True)
        request = scene.to_request(self.client.ros_distro)
        self.APPLY_PLANNING_SCENE(self.client, request, callback, errback)

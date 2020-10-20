from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import AppendCollisionMesh
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    'MoveItAppendCollisionMesh',
]


class MoveItAppendCollisionMesh(AppendCollisionMesh):
    """Callable to append a collision mesh to the planning scene."""
    APPLY_PLANNING_SCENE = ServiceDescription('/apply_planning_scene',
                                              'ApplyPlanningScene',
                                              ApplyPlanningSceneRequest,
                                              ApplyPlanningSceneResponse,
                                              )

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def append_collision_mesh(self, collision_mesh, options=None):
        """Append a collision mesh to the planning scene.

        Parameters
        ----------
        collision_mesh : :class:`compas_fab.robots.CollisionMesh`
            Object containing the collision mesh to be appended.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        kwargs = {}
        kwargs['collision_mesh'] = collision_mesh
        kwargs['errback_name'] = 'errback'

        return await_callback(self.append_collision_mesh_async, **kwargs)

    def append_collision_mesh_async(self, callback, errback, collision_mesh):
        co = CollisionObject.from_collision_mesh(collision_mesh)
        co.operation = CollisionObject.APPEND
        world = PlanningSceneWorld(collision_objects=[co])
        scene = PlanningScene(world=world, is_diff=True)
        request = dict(scene=scene)
        self.APPLY_PLANNING_SCENE(self.ros_client, request, callback, errback)

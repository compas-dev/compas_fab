from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import RemoveAttachedCollisionMesh
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    'MoveItRemoveAttachedCollisionMesh',
]


class MoveItRemoveAttachedCollisionMesh(RemoveAttachedCollisionMesh):
    """Callable to remove an attached collision mesh from the robot."""
    APPLY_PLANNING_SCENE = ServiceDescription('/apply_planning_scene',
                                              'ApplyPlanningScene',
                                              ApplyPlanningSceneRequest,
                                              ApplyPlanningSceneResponse,
                                              )

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def remove_attached_collision_mesh(self, id, options=None):
        """Remove an attached collision mesh from the robot.

        Parameters
        ----------
        id : str
            Name of collision mesh to be removed.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        kwargs = {}
        kwargs['id'] = id
        kwargs['errback_name'] = 'errback'

        return await_callback(self.remove_attached_collision_mesh_async, **kwargs)

    def remove_attached_collision_mesh_async(self, callback, errback, id):
        aco = AttachedCollisionObject()
        aco.object.id = id
        aco.object.operation = CollisionObject.REMOVE
        world = PlanningSceneWorld(collision_objects=[aco.object])
        scene = PlanningScene(world=world, is_diff=True)
        request = dict(scene=scene)
        self.APPLY_PLANNING_SCENE(self.ros_client, request, callback, errback)

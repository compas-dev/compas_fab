from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import AddAttachedCollisionMesh
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import AttachedCollisionObject
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import PlanningScene
from compas_fab.backends.ros.messages import PlanningSceneWorld
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    'MoveItAddAttachedCollisionMesh',
]


class MoveItAddAttachedCollisionMesh(AddAttachedCollisionMesh):
    """Callable to add a collision mesh and attach it to the robot."""
    APPLY_PLANNING_SCENE = ServiceDescription('/apply_planning_scene',
                                              'ApplyPlanningScene',
                                              ApplyPlanningSceneRequest,
                                              ApplyPlanningSceneResponse,
                                              )

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def add_attached_collision_mesh(self, attached_collision_mesh, options=None):
        """Add a collision mesh and attach it to the robot.

        Parameters
        ----------
        attached_collision_mesh : :class:`compas_fab.robots.AttachedCollisionMesh`
            Object containing the collision mesh to be attached.
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        kwargs = {}
        kwargs['attached_collision_mesh'] = attached_collision_mesh
        kwargs['errback_name'] = 'errback'

        return await_callback(self.add_attached_collision_mesh_async, **kwargs)

    def add_attached_collision_mesh_async(self, callback, errback, attached_collision_mesh):
        aco = AttachedCollisionObject.from_attached_collision_mesh(
            attached_collision_mesh)
        aco.object.operation = CollisionObject.ADD
        world = PlanningSceneWorld(collision_objects=[aco.object])
        scene = PlanningScene(world=world, is_diff=True)
        request = dict(scene=scene)
        self.APPLY_PLANNING_SCENE(self.ros_client, request, callback, errback)

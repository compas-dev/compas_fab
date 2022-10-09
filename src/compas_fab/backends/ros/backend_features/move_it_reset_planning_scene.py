from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import ResetPlanningScene
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    'MoveItResetPlanningScene',
]


class MoveItResetPlanningScene(ResetPlanningScene):
    """Callable to add a collision mesh to the planning scene."""

    APPLY_PLANNING_SCENE = ServiceDescription(
        '/apply_planning_scene',
        'ApplyPlanningScene',
        ApplyPlanningSceneRequest,
        ApplyPlanningSceneResponse,
    )

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def reset_planning_scene(self, options=None):
        """Resets the planning scene, removing all added collision meshes.

        Parameters
        ----------
        options : dict, optional
            Unused parameter.

        Returns
        -------
        ``None``
        """
        kwargs = {}
        kwargs['errback_name'] = 'errback'

        return await_callback(self.reset_planning_scene_async, **kwargs)

    def reset_planning_scene_async(self, callback, errback):
        scene = self.ros_client.get_planning_scene()
        for collision_object in scene.world.collision_objects:
            collision_object.operation = CollisionObject.REMOVE
        for collision_object in scene.robot_state.attached_collision_objects:
            collision_object.object['operation'] = CollisionObject.REMOVE
        scene.is_diff = True
        scene.robot_state.is_diff = True
        request = scene.to_request(self.ros_client.ros_distro)
        self.APPLY_PLANNING_SCENE(self.ros_client, request, callback, errback)

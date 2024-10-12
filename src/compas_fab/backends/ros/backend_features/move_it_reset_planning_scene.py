from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.interfaces import ResetPlanningScene
from compas_fab.backends.ros.messages import ApplyPlanningSceneRequest
from compas_fab.backends.ros.messages import ApplyPlanningSceneResponse
from compas_fab.backends.ros.messages import CollisionObject
from compas_fab.backends.ros.messages import PlanningScene  # noqa: F401
from compas_fab.backends.ros.service_description import ServiceDescription

__all__ = [
    "MoveItResetPlanningScene",
]


class MoveItResetPlanningScene(ResetPlanningScene):
    """Callable to add a collision mesh to the planning scene."""

    APPLY_PLANNING_SCENE = ServiceDescription(
        "/apply_planning_scene",
        "ApplyPlanningScene",
        ApplyPlanningSceneRequest,
        ApplyPlanningSceneResponse,
    )

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
        kwargs["errback_name"] = "errback"

        # The complete removal of the planning scene is done in two steps because
        # the attached collision objects need to be removed before the collision objects
        # can be removed.
        step_1_result = await_callback(self._remove_aco_async, **kwargs)
        assert step_1_result.success, "Failed to remove attached collision objects"
        step_2_result = await_callback(self._remove_co_async, **kwargs)
        assert step_2_result.success, "Failed to remove collision objects"

        return (step_1_result, step_2_result)

    def _remove_aco_async(self, callback, errback):
        scene = self.get_planning_scene()  # type: PlanningScene
        for attached_collision_object in scene.robot_state.attached_collision_objects:
            attached_collision_object.object["operation"] = CollisionObject.REMOVE
        scene.is_diff = True
        scene.robot_state.is_diff = True
        request = scene.to_request(self.client.ros_distro)
        self.APPLY_PLANNING_SCENE(self.client, request, callback, errback)

    def _remove_co_async(self, callback, errback):
        scene = self.get_planning_scene()  # type: PlanningScene
        for collision_object in scene.world.collision_objects:
            collision_object.operation = CollisionObject.REMOVE
        scene.is_diff = True
        scene.robot_state.is_diff = True
        request = scene.to_request(self.client.ros_distro)
        self.APPLY_PLANNING_SCENE(self.client, request, callback, errback)

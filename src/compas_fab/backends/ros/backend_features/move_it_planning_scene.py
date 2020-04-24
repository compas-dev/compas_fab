from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.utilities import await_callback

from compas_fab.backends.ros.messages import GetPlanningSceneRequest
from compas_fab.backends.ros.messages import GetPlanningSceneResponse
from compas_fab.backends.ros.messages import PlanningSceneComponents
from compas_fab.backends.ros.planner_backend import ServiceDescription


class MoveItPlanningScene(object):
    GET_PLANNING_SCENE = ServiceDescription('/get_planning_scene',
                                            'GetPlanningScene',
                                            GetPlanningSceneRequest,
                                            GetPlanningSceneResponse)

    def __init__(self, ros_client):
        self.ros_client = ros_client

    def __call__(self, *args, **kwargs):
        return self.get_planning_scene()

    def get_planning_scene(self):
        kwargs = {}
        kwargs['errback_name'] = 'errback'

        return await_callback(self.get_planning_scene_async, **kwargs)

    def get_planning_scene_async(self, callback, errback):
        request = dict(components=PlanningSceneComponents(PlanningSceneComponents.SCENE_SETTINGS |
                                                          PlanningSceneComponents.ROBOT_STATE |
                                                          PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                                                          PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                                          PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                                                          PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                                                          PlanningSceneComponents.OBJECT_COLORS))
        self.GET_PLANNING_SCENE(self.ros_client, request, callback, errback)

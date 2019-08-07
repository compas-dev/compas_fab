import time

from compas_fab.backends import RosClient
from compas_fab.robots import PlanningScene
from compas_fab.robots.ur5 import Robot

with RosClient() as client:
    robot = Robot(client)
    scene = PlanningScene(robot)
    scene.remove_collision_mesh('floor')

    # sleep a bit before terminating the client
    time.sleep(1)

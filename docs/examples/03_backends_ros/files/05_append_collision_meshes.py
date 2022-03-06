import time

from compas.datastructures import Mesh
from compas.geometry import Box

from compas_fab.backends import RosClient
from compas_fab.robots import CollisionMesh
from compas_fab.robots import PlanningScene

with RosClient() as client:
    robot = client.load_robot()
    scene = PlanningScene(robot)
    assert robot.name == 'ur5_robot'

    brick = Box.from_width_height_depth(0.11, 0.07, 0.25)

    for i in range(5):
        mesh = Mesh.from_vertices_and_faces(brick.vertices, brick.faces)
        cm = CollisionMesh(mesh, 'brick')
        cm.frame.point.y += 0.5
        cm.frame.point.z += brick.zsize * i

        scene.append_collision_mesh(cm)

    # sleep a bit before removing the bricks
    time.sleep(1)

    scene.remove_collision_mesh('brick')

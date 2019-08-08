from __future__ import print_function
import os
import time
import pytest

from compas.datastructures import Mesh
from compas.geometry import Frame

import compas_fab
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots.ur5 import Robot
from compas_fab.robots import CollisionMesh
from compas_fab.backends import RosClient
from compas_fab.robots import PlanningScene

def test_get_collision_objects_from_planning_scene():
    with RosClient() as client:
        assert client.is_connected
        robot = Robot(client)

        scene = PlanningScene(robot)
        mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        cm = CollisionMesh(mesh, 'floor')
        scene.add_collision_mesh(cm)

        # See: https://github.com/compas-dev/compas_fab/issues/63#issuecomment-519525879
        time.sleep(1)

        co_dict = client.get_collision_meshes_and_poses()
        assert len(co_dict) == 1 and 'floor' in co_dict
        assert len(co_dict['floor']['meshes']) == 1
        assert len(co_dict['floor']['mesh_poses']) == 1
        floor_mesh = co_dict['floor']['meshes'][0]
        assert floor_mesh.number_of_vertices() == 4
        assert floor_mesh.number_of_faces() == 2
        assert floor_mesh.number_of_edges() == 5

        floor_frame = co_dict['floor']['mesh_poses'][0]
        assert floor_frame == Frame.worldXY()

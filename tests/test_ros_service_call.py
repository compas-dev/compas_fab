from __future__ import print_function
import os
import time
import pytest
import math
import numpy as np

from compas.datastructures import Mesh
from compas.geometry import Frame

import compas_fab
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots.ur5 import Robot
from compas_fab.robots import CollisionMesh
from compas_fab.backends import RosClient
from compas_fab.robots import PlanningScene
from compas_fab.robots import Configuration

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


def test_single_query_motion_plan_from_ee_frame():
    with RosClient() as client:
        robot = Robot(client)

        frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        tolerance_position = 0.001
        tolerance_axes = [np.radians(1)] * 3

        start_configuration = Configuration.from_revolute_values([-0.042, 4.295, 0, -3.327, 4.755, 0.])
        group = robot.main_group_name

        # create goal constraints from frame
        goal_constraints = robot.constraints_from_frame(frame,
                                                        tolerance_position,
                                                        tolerance_axes,
                                                        group)

        trajectory = robot.plan_motion(goal_constraints,
                                       start_configuration,
                                       group,
                                       planner_id='RRT')

        print("Motion plan from frame: computed kinematic path with %d configurations." % len(trajectory.points))
        print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)


@pytest.mark.mm
def test_single_query_motion_plan_from_joint_conf():
    with RosClient() as client:
        robot = Robot(client)
        group = robot.main_group_name

        scene = PlanningScene(robot)
        scene.remove_all_collision_objects()
        current_jt_state = scene.get_joint_state()

        ur5_idle_conf = np.array([104., -80., -103., -86., 89., 194.]) / 180.0 * np.pi
        st_conf = Configuration.from_revolute_values(current_jt_state.values())
        goal_conf = Configuration.from_revolute_values(list(ur5_idle_conf))

        # create goal constraints from joint conf
        goal_constraints = robot.constraints_from_configuration(goal_conf, [0.001], group)
        traj = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRT')

        print("Motion plan from joint: computed kinematic path with %d configurations." % len(traj.points))
        print("Executing this path at full speed would take approx. %.3f seconds." % traj.time_from_start)


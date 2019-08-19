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


# @pytest.mark.mm
def test_single_query_motion_plan_from_ee_frame():
    with RosClient() as client:
        robot = Robot(client)

        frame = Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1])
        tolerance_position = 0.001
        tolerance_axes = [np.radians(1)] * 3

        ur5_idle_conf = np.array([104., -80., -103., -86., 89., 194.]) / 180.0 * np.pi
        goal_conf_val = [3.3692990008117247, 4.006257566491165, 4.70930506391325,
                         3.8508079839547396, 3.369299000811508, 3.1415926535897873]
        start_configuration = Configuration.from_revolute_values(goal_conf_val)
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


# @pytest.mark.mm
def test_single_query_motion_plan_from_joint_conf():
    with RosClient() as client:
        robot = Robot(client)
        group = robot.main_group_name
        print(robot.info())

        scene = PlanningScene(robot)
        scene.remove_all_collision_objects()
        current_jt_state = scene.get_joint_state()

        # ur5_idle_conf = np.array([104., -80., -103., -86., 89., 194.]) / 180.0 * np.pi
        # st_conf = Configuration.from_revolute_values(current_jt_state.values())
        # goal_conf = Configuration.from_revolute_values(list(ur5_idle_conf))

        st_conf_val = [1.8151424220741026, -1.3962634015954636, -1.7976891295541593,
                       -1.5009831567151235, 1.5533430342749532, 3.385938748868999]
        goal_conf_val = [3.3692990008117247, 4.006257566491165, 4.70930506391325,
                         3.8508079839547396, 3.369299000811508, 3.1415926535897873]
        st_conf = Configuration.from_revolute_values(st_conf_val)
        goal_conf = Configuration.from_revolute_values(goal_conf_val)

        # create goal constraints from joint conf
        goal_constraints = robot.constraints_from_configuration(goal_conf, [math.radians(5)] * 6, group)
        traj = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRT ',
                                 num_planning_attempts=20, allowed_planning_time=10)

        print("Motion plan from joint: computed kinematic path with %d configurations." % len(traj.points))
        print("Executing this path at full speed would take approx. %.3f seconds." % traj.time_from_start)


@pytest.mark.mm
def test_is_joint_state_valid():
    with RosClient() as client:
        robot = Robot(client)
        group = robot.main_group_name
        joint_names = robot.get_configurable_joint_names()

        idle_conf_val = [1.8151424220741026, -1.3962634015954636, -1.7976891295541593,
                       -1.5009831567151235, 1.5533430342749532, 3.385938748868999]
        self_collision_conf_val = [0.05092512883900069, 4.407050465744106, 3.5727222613517697,
                                   1.4450052336734978, 4.661463851545683, 0.0]
        print(client.is_joint_state_colliding(group, joint_names, idle_conf_val))
        print(client.is_joint_state_colliding(group, joint_names, self_collision_conf_val))

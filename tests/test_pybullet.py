from __future__ import print_function
import os
import time
import pytest
from numpy.testing import assert_array_almost_equal

from compas.geometry import Frame
from compas.geometry import Translation
from compas.datastructures import Mesh
from compas.robots import LocalPackageMeshLoader
from compas.robots import RobotModel

import compas_fab
from compas_fab.backends import RosClient
from compas_fab.robots import Robot as RobotClass
from compas_fab.robots import RobotSemantics
from compas_fab.robots import PlanningScene
from compas_fab.robots import CollisionMesh
from compas_fab.robots.ur5 import Robot

from compas_fab.backends.pybullet import attach_end_effector_geometry, \
convert_mesh_to_pybullet_body, get_TCP_pose, create_pb_robot_from_ros_urdf

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint,\
    create_obj, set_pose


def test_convert_compas_robot_to_pybullet_robot():
    # get ur robot model from local test data that's shipped with compas_fab
    # does not need client connection here

    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
    ee_filename = compas_fab.get('universal_robot/ur_description/meshes/pychoreo_workshop_gripper/collision/pychoreo-workshop-gripper.stl')
    urdf_pkg_name = 'ur_description'

    # geometry file is not loaded here
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
    robot = RobotClass(model, semantics=semantics)
    ee_link_name = robot.get_end_effector_link_name()
    # print('ee link: {}'.format(ee_link_name))

    # parse end effector mesh
    ee_mesh = Mesh.from_stl(ee_filename)

    # define TCP transformation
    tcp_tf = Translation([0.2, 0, 0]) # in meters

    connect(use_gui=False)
    # if the following returns without error, we are good
    pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name)

    # get disabled collisions
    disabled_collisions = semantics.get_disabled_collisions()
    assert len(disabled_collisions) == 10
    assert ('base_link', 'shoulder_link') in disabled_collisions

    # attach tool
    ee_bodies = attach_end_effector_geometry([ee_mesh], pb_robot, ee_link_name)

    # draw TCP frame in pybullet
    TCP_pb_pose = get_TCP_pose(pb_robot, ee_link_name, tcp_tf, return_pb_pose=True)
    draw_pose(TCP_pb_pose, length=0.04)

    # wait_for_user()


def test_convert_planning_scene_collision_objects_to_pybullet_obstacles():
    with RosClient() as client:
        assert client.is_connected
        robot = Robot(client)

        scene = PlanningScene(robot)
        mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
        cm = CollisionMesh(mesh, 'floor')
        scene.add_collision_mesh(cm)

        # See: https://github.com/compas-dev/compas_fab/issues/63#issuecomment-519525879
        time.sleep(1)

        connect(use_gui=False)
        co_dict = client.get_collision_meshes_and_poses()
        body_dict = {}
        for name, item_dict in co_dict.items():
            n_obj = len(item_dict['meshes'])
            for i, mesh, frame in zip(range(n_obj), item_dict['meshes'], item_dict['mesh_poses']):
                body_name = name + '_' + str(i)
                body = convert_mesh_to_pybullet_body(mesh, frame, body_name)
                body_dict[body_name] = body

        assert len(body_dict) == 1
        pyb_pose = get_pose(list(body_dict.values())[0])
        input_frame = list(co_dict.values())[0]['mesh_poses'][0]
        assert pyb_pose[0] == input_frame.point
        assert_array_almost_equal(euler_from_quat(pyb_pose[1]), input_frame.euler_angles())
        # wait_for_user()

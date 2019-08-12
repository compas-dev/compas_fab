from __future__ import print_function
import os
import time
import pytest

import numpy as np
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
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots.ur5 import Robot
from compas_fab.robots.configuration import Configuration

from compas_fab.backends.pybullet import attach_end_effector_geometry, \
convert_mesh_to_pybullet_body, get_TCP_pose, create_pb_robot_from_ros_urdf, \
convert_meshes_and_poses_to_pybullet_bodies

from compas_fab.backends.ros.plugins_choreo import load_pick_and_place

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint,\
    create_obj, set_pose, joints_from_names, set_joint_positions, get_fixed_constraints

@pytest.mark.wip
def test_choreo_plan_single_cartesian_motion():
    choreo_problem_instance_dir = compas_fab.get('choreo_instances')
    unit_geos, static_obstacles = load_pick_and_place(choreo_problem_instance_dir,
                                                      'ur_picknplace_single_piece', scale=1e-3)

    with RosClient() as client:
        urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
        srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')
        ee_filename = compas_fab.get('universal_robot/ur_description/meshes/' +
                                     'pychoreo_workshop_gripper/collision/victor_gripper_jaw03.obj')
        urdf_pkg_name = 'ur_description'

        # geometry file is not loaded here
        model = RobotModel.from_urdf_file(urdf_filename)
        semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
        robot = RobotClass(model, semantics=semantics, client=client)

        base_link_name = robot.get_base_link_name()
        ee_link_name = robot.get_end_effector_link_name()
        ik_joint_names = robot.get_configurable_joint_names()

        # TODO: attach end effector to the robot in planning scene
        # https://github.com/compas-dev/compas_fab/issues/66
        scene = PlanningScene(robot)

        # parse end effector mesh
        ee_mesh = Mesh.from_obj(ee_filename)
        # ee_mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        # cm = CollisionMesh(ee_mesh, 'gripper')
        # touch_links = ['wrist_3_link', ee_link_name]
        # acm = AttachedCollisionMesh(cm, ee_link_name, touch_links)
        # scene.add_attached_collision_mesh(acm)

        # define TCP transformation
        tcp_tf = Translation([0.099, 0, 0]) # in meters

        ur5_start_conf = np.array([104., -80., -103., -86., 89., 194.]) / 180.0 * np.pi

        # group = robot.main_group_name
        # current_jt_state = scene.get_joint_state()
        # st_conf = Configuration.from_revolute_values(current_jt_state.values())
        # goal_conf = Configuration.from_revolute_values(list(ur5_start_conf))
        # goal_constraints = robot.constraints_from_configuration(goal_conf, 0.001, group)
        # reset_traj = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRT')
        # TODO: execute it!
        # TODO: or simply: client.set_joint_positions(group, ur5_start_conf)

        # add static collision obstacles
        for static_obs_name, static_obs_mesh in static_obstacles.items():
            cm = CollisionMesh(static_obs_mesh, static_obs_name)
            scene.add_collision_mesh(cm)

        for unit_name, unit_geo in unit_geos.items():
            for sub_id, mesh in enumerate(unit_geo.mesh):
                cm = CollisionMesh(mesh, unit_name + '_' + str(sub_id), frame=unit_geo.initial_frame)
                scene.add_collision_mesh(cm)

        # See: https://github.com/compas-dev/compas_fab/issues/63#issuecomment-519525879
        time.sleep(1)

        # start pybullet environment & load pybullet robot
        connect(use_gui=True)
        pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name,
                                                 planning_scene=scene,
                                                 ee_link_name=ee_link_name)
        ee_bodies = attach_end_effector_geometry([ee_mesh], pb_robot, ee_link_name)

        pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)
        set_joint_positions(pb_robot, pb_ik_joints, ur5_start_conf)

        # draw TCP frame in pybullet
        if has_gui():
            TCP_pb_pose = get_TCP_pose(pb_robot, ee_link_name, tcp_tf, return_pb_pose=True)
            handles = draw_pose(TCP_pb_pose, length=0.04)

        # deliver ros collision meshes to pybullet
        # TODO: this co_dict shouldn't be a ROS message
        co_dict = scene.get_collision_meshes_and_poses()
        body_dict = convert_meshes_and_poses_to_pybullet_bodies(co_dict)

        wait_for_user()

        scene.remove_all_collision_objects()

    # frames = []
    # frames.append(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]))
    # frames.append(Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1]))
    # start_configuration = Configuration.from_revolute_values([-0.042, 0.033, -2.174, 5.282, -1.528, 0.000])

    # # trajectory = robot.plan_cartesian_motion(frames,
    # #                                          start_configuration,
    # #                                          max_step=0.01,
    # #                                          avoid_collisions=True)

    # print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    # print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    # print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

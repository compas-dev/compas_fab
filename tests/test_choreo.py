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
convert_meshes_and_poses_to_pybullet_bodies, sanity_check_collisions, \
pb_pose_from_Transformation

from compas_fab.backends.ros.plugins_choreo import load_pick_and_place, display_picknplace_trajectories

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat, draw_pose, \
    get_link_pose, link_from_name, create_attachment, add_fixed_constraint,\
    create_obj, set_pose, joints_from_names, set_joint_positions, get_fixed_constraints, \
    remove_debug
from choreo import direct_ladder_graph_solve_picknplace, divide_nested_list_chunks

import ikfast_ur5

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
        scene.remove_all_collision_objects()

        # parse end effector mesh
        ee_mesh = Mesh.from_obj(ee_filename)

        # define TCP transformation
        tcp_tf = Translation([0.099, 0, 0]) # in meters
        ur5_start_conf = np.array([104., -80., -103., -86., 89., 194.]) / 180.0 * np.pi

        # add static collision obstacles
        for static_obs_name, static_obs_mesh in static_obstacles.items():
            # offset the table a bit...
            cm = CollisionMesh(static_obs_mesh, static_obs_name, frame=Frame.from_transformation(Translation([0, 0, -0.02])))
            scene.add_collision_mesh(cm)

        # See: https://github.com/compas-dev/compas_fab/issues/63#issuecomment-519525879
        time.sleep(1)

        # start pybullet environment & load pybullet robot
        connect(use_gui=True)
        pb_robot = create_pb_robot_from_ros_urdf(urdf_filename, urdf_pkg_name,
                                                 planning_scene=scene,
                                                 ee_link_name=ee_link_name)
        ee_attachs = attach_end_effector_geometry([ee_mesh], pb_robot, ee_link_name)

        # update current joint conf and attach end effector
        pb_ik_joints = joints_from_names(pb_robot, ik_joint_names)
        set_joint_positions(pb_robot, pb_ik_joints, ur5_start_conf)
        for e_at in ee_attachs: e_at.assign()

        # draw TCP frame in pybullet
        if has_gui():
            TCP_pb_pose = get_TCP_pose(pb_robot, ee_link_name, tcp_tf, return_pb_pose=True)
            handles = draw_pose(TCP_pb_pose, length=0.04)

        # deliver ros collision meshes to pybullet
        # TODO: this co_dict shouldn't be a ROS message
        co_dict = scene.get_collision_meshes_and_poses()
        static_obstacles_from_name = convert_meshes_and_poses_to_pybullet_bodies(co_dict)
        # for now...
        for so_key, so_val in static_obstacles_from_name.items():
            static_obstacles_from_name[so_key] = so_val[0]
        print(static_obstacles_from_name)

        for unit_name, unit_geo in unit_geos.items():
            geo_bodies = []
            for sub_id, mesh in enumerate(unit_geo.mesh):
                geo_bodies.append(convert_mesh_to_pybullet_body(mesh))
            unit_geo.pybullet_bodies = geo_bodies

        # check collision between obstacles and element geometries
        assert not sanity_check_collisions(unit_geos, static_obstacles_from_name)

        from random import shuffle
        seq_assignment = list(range(len(unit_geos)))
        # shuffle(seq_assignment)
        element_seq = {seq_id : e_id for seq_id, e_id in enumerate(seq_assignment)}

        for key, val in element_seq.items():
            element_seq[key] = 'e_' + str(val)

        if has_gui():
            handles = []
            for e_id in element_seq.values():
                # for e_body in brick_from_index[e_id].body: set_pose(e_body, brick_from_index[e_id].goal_pose)
                handles.extend(draw_pose(unit_geos[e_id].initial_pb_pose, length=0.02))
                handles.extend(draw_pose(unit_geos[e_id].goal_pb_pose, length=0.02))
                for e_body in unit_geos[e_id].pybullet_bodies:
                    set_pose(e_body, unit_geos[e_id].initial_pb_pose)
            print('pybullet env loaded.')
            wait_for_user()
            for h in handles:
                remove_debug(h)

        ik_fn = ikfast_ur5.get_ik
        tot_traj, graph_sizes = \
        direct_ladder_graph_solve_picknplace(pb_robot, ik_joint_names, base_link_name, ee_link_name, ik_fn,
            unit_geos, element_seq, static_obstacles_from_name,
            tcp_transf=pb_pose_from_Transformation(tcp_tf),
            ee_attachs=ee_attachs,
            max_attempts=100, viz=True)

        picknplace_cart_plans = divide_nested_list_chunks(tot_traj, graph_sizes)
        print(picknplace_cart_plans)
        print('Cartesian planning finished.')
        if has_gui():
            wait_for_user()

        print('\n*************************\nplanning completed. Simulate?')
        if has_gui():
            wait_for_user()
        for e_id in element_seq.values():
            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].initial_pb_pose)

        display_picknplace_trajectories(pb_robot, ik_joint_names, ee_link_name,
                                        unit_geos, element_seq, picknplace_cart_plans, \
                                        ee_attachs=ee_attachs,
                                        cartesian_time_step=0.075, transition_time_step=0.1, step_sim=True)

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

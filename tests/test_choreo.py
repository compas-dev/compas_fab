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
from compas_fab.robots import CollisionMesh, AttachedCollisionMesh
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

# TODO: test ignore if ros client is not running
@pytest.mark.wip
def test_choreo_plan_single_cartesian_motion():
    VIZ=False

    choreo_problem_instance_dir = compas_fab.get('choreo_instances')
    unit_geos, static_obstacles = load_pick_and_place(choreo_problem_instance_dir,
                                                        'ur_picknplace_single_piece', scale=1e-3)
    # unit_geos, static_obstacles = load_pick_and_place(choreo_problem_instance_dir,
    #                                                     'ur_picknplace_multiple_piece', scale=1e-3)

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

        # from random import shuffle
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
            max_attempts=100, viz=VIZ)

        picknplace_cart_plans = divide_nested_list_chunks(tot_traj, graph_sizes)
        print(picknplace_cart_plans)
        print('Cartesian planning finished.')

        # reset robot and parts for better visualization
        set_joint_positions(pb_robot, pb_ik_joints, ur5_start_conf)
        for ee in ee_attachs: ee.assign()
        for e_id in element_seq.values():
            for e_body in unit_geos[e_id].pybullet_bodies:
                set_pose(e_body, unit_geos[e_id].initial_pb_pose)

        if has_gui():
            wait_for_user()

        print('Transition planning started.')

        group = robot.main_group_name
        for seq_id, unit_picknplace in enumerate(picknplace_cart_plans):
            print('transition seq#{}'.format(seq_id))

            if seq_id != 0:
                tr_start_conf = picknplace_cart_plans[seq_id-1]['place_retreat'][-1]
            else:
                tr_start_conf = ur5_start_conf
            # set_joint_positions(robot, movable_joints, tr_start_conf)


            # cur_mo_list = []
            # for mo_id, mo in brick_from_index.items():
            #     if mo_id in element_seq.values():
            #         cur_mo_list.extend(mo.body)

            # obstacles=static_obstacles + cur_mo_list
            st_conf = Configuration.from_revolute_values(tr_start_conf)
            goal_conf = Configuration.from_revolute_values(picknplace_cart_plans[seq_id]['pick_approach'][0])
            goal_constraints = robot.constraints_from_configuration(goal_conf, [0.001], group)
            place2pick_path = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRT')

            # # create attachement without needing to keep track of grasp...
            # set_joint_positions(robot, movable_joints, picknplace_cart_plans[seq_id]['pick_retreat'][0])
            # # attachs = [Attachment(robot, tool_link, invert(grasp.attach), e_body) for e_body in brick.body]
            # attachs = [create_attachment(robot, end_effector_link, e_body) for e_body in brick_from_index[e_id].body]

            # cur_mo_list = []
            # for mo_id, mo in brick_from_index.items():
            #     if mo_id != e_id and mo_id in element_seq.values():
            #         cur_mo_list.extend(mo.body)

            st_conf = Configuration.from_revolute_values(picknplace_cart_plans[seq_id]['pick_retreat'][-1])
            goal_conf = Configuration.from_revolute_values(picknplace_cart_plans[seq_id]['place_approach'][0])
            goal_constraints = robot.constraints_from_configuration(goal_conf, [0.001], group)
            pick2place_path = robot.plan_motion(goal_constraints, st_conf, group, planner_id='RRT')

            picknplace_cart_plans[seq_id]['place2pick'] = [traj_pt.positions for traj_pt in place2pick_path]
            picknplace_cart_plans[seq_id]['pick2place'] = [traj_pt.positions for traj_pt in pick2place_path]

        print('Transition planning finished.')

        print('\n*************************\nplanning completed. Simulate?')
        if has_gui():
            wait_for_user()

        display_picknplace_trajectories(pb_robot, ik_joint_names, ee_link_name,
                                        unit_geos, element_seq, picknplace_cart_plans, \
                                        ee_attachs=ee_attachs,
                                        cartesian_time_step=0.075, transition_time_step=0.1, step_sim=True)

        scene.remove_all_collision_objects()

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


# def test_choreo_plan_single_cartesian_motion():
    ## this test should not reply on client's existence
    # robot = Robot(client)
    #
    # frames = []
    # frames.append(Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0]))
    # frames.append(Frame([0.4, 0.3, 0.4], [0, 1, 0], [0, 0, 1]))
    # start_configuration = Configuration.from_revolute_values([-0.042, 0.033, -2.174, 5.282, -1.528, 0.000])
    #
    # # trajectory = robot.plan_cartesian_motion(frames,
    # #                                          start_configuration,
    # #                                          max_step=0.01,
    # #                                          avoid_collisions=True)
    #
    # print("Computed cartesian path with %d configurations, " % len(trajectory.points))
    # print("following %d%% of requested trajectory." % (trajectory.fraction * 100))
    # print("Executing this path at full speed would take approx. %.3f seconds." % trajectory.time_from_start)

from __future__ import print_function
import os
import time
import pytest
from numpy.testing import assert_array_almost_equal

from compas.geometry import Frame
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

from compas_fab.backends.ros.plugins_choreo import ChoreoPlanner
from compas_fab.backends.ros.plugins_choreo import generate_rel_path_URDF_pkg
from compas_fab.backends.ros.plugins_choreo import convert_mesh_to_pybullet_body

from conrob_pybullet import load_pybullet, connect, disconnect, wait_for_user, \
    LockRenderer, has_gui, get_model_info, get_pose, euler_from_quat

def test_convert_compas_robot_to_pybullet_robot():
    # get ur robot model from local test data that's shipped with compas_fab
    # does not need client connection here
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

    # geometry file is not loaded here
    model = RobotModel.from_urdf_file(urdf_filename)
    rel_urdf_path = generate_rel_path_URDF_pkg(urdf_filename, 'ur_description')
    print('\nURDF with relative path generated: {}'.format(rel_urdf_path))

    try:
        connect(use_gui=False)
        # if the following returns without pybullet.error
        # we are good
        pb_robot = load_pybullet(rel_urdf_path, fixed_base=True)

        # get disabled collisions
        semantics = RobotSemantics.from_srdf_file(srdf_filename, model)
        disabled_collisions = semantics.get_disabled_collisions()
        assert len(disabled_collisions) == 10
        assert ('base_link', 'shoulder_link') in disabled_collisions

    except:
        os.remove(rel_urdf_path)

    os.remove(rel_urdf_path)


@pytest.mark.wip
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
        print(co_dict)
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

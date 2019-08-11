"""utililty functions for grasps and picknplace planning in pybullet.

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas_fab.backends.ros.plugins_choreo import sample_tool_ik, best_sol

from conrob_pybullet import BodyPose, BodyConf, Command, \
    get_free_motion_gen, get_holding_motion_gen, BodyPath, Attach

from conrob_pybullet import WorldSaver, connect, dump_world, set_pose, \
    stable_z, BLOCK_URDF, load_model, wait_for_interrupt, \
    disconnect, user_input, update_state, disable_real_time, load_pybullet, \
    draw_pose, get_movable_joints, set_joint_positions, enable_gravity, \
    end_effector_from_body, approach_from_grasp, inverse_kinematics, \
    pairwise_collision, get_sample_fn, plan_direct_joint_motion, \
    HideOutput, joints_from_names, link_from_name, multiply, invert, has_gui, \
    wait_for_user, remove_debug, get_joint_positions
from conrob_pybullet import Pose, Point, BodyGrasp

def get_grasp_ik_fn(ik_fn, robot, ik_joint_names, base_link_name,
                    fixed=[], num_attempts=10, self_collisions=True, teleport=False):
    ik_joints = joints_from_names(robot, ik_joint_names)
    def ik_for_grasp_fn(body, body_pose, body_grasp, extra_tcp_tf_fn=None):
        """get ik solutions for grasp the body object in pose with given grasp info

        """
        obstacles = [body] + fixed
        world_from_gripper = multiply(body_pose.pose,
                                      invert(body_grasp.grasp_pose))
        world_from_approach = multiply(world_from_gripper,
                                       body_grasp.approach_pose)
        if extra_tcp_tf_fn:
            world_from_gripper = extra_tcp_tf_fn(world_from_gripper)
            world_from_approach = extra_tcp_tf_fn(world_from_approach)

        if has_gui():
            h1 = draw_pose(world_from_gripper, length=0.04)
            h2 = draw_pose(world_from_approach, length=0.02)
            wait_for_user()
            for h in h1+h2: remove_debug(h)

        for _ in range(num_attempts):
            # nearest conf will be returned by default
            q_approach_sols = sample_tool_ik(ik_fn, robot, ik_joint_names, \
                                             base_link_name, world_from_approach,
                                             get_all=True)
            current_conf = get_joint_positions(robot, ik_joints)
            q_approach = best_sol(q_approach_sols, current_conf, [1.]*6)
            if q_approach is not None:
                set_joint_positions(robot, ik_joints, q_approach)
                wait_for_user()

            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                # TODO: gripper world collision check...
                continue
            conf = BodyConf(robot, joints=ik_joints)

            q_grasp_sols = sample_tool_ik(ik_fn, robot, ik_joint_names, \
                                          base_link_name, world_from_approach,
                                          get_all=True)
            current_conf = get_joint_positions(robot, ik_joints)
            q_grasp = best_sol(q_grasp_sols, current_conf, [1.]*6)
            if q_grasp is not None:
                set_joint_positions(robot, ik_joints, q_grasp)

            if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue

            conf.assign()
            wait_for_user()
            path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles, \
                                            self_collisions=self_collisions)
            if path is None:
                continue
            command = Command([BodyPath(robot, path, joints=ik_joints),
                               Attach(body, robot, body_grasp.link),
                               BodyPath(robot, path[::-1], joints=ik_joints,
                                        attachments=[body_grasp])])
            return (conf, command)
            # TODO: holding collisions
        return None
    return ik_for_grasp_fn


def get_grasp_gen(robot, tool_frame_name, grasp_info, tool_from_tcp=None):
    """Get a grasp generator.

    Parameters
    ----------
    robot : type
    tool_frame_name : str
        e.g. 'ee_link'
    grasp_info: GraspInfo
        namedtuple('GraspInfo', ['get_grasps', 'approach_pose'])
    tool_from_tcp : Pose
        extra ee_mount_link to tcp transformation if any, defaults to None

    Returns
    -------
    function handle
        gen(pybullet body) : BodyGrasp

    """
    end_effector_link = link_from_name(robot, tool_frame_name)
    def gen(body):
        grasp_poses = grasp_info.get_grasps(body)
        for tool_from_body in grasp_poses:
            # approach_pose = grasp_from_approach
            if tool_from_tcp:
                tcp_from_body = multiply(invert(tool_from_tcp), tool_from_body)
            else:
                tcp_from_body = tool_from_body
            body_grasp = BodyGrasp(body, tcp_from_body, grasp_info.approach_pose,
                                   robot, end_effector_link)
            yield (body_grasp,)
    return gen


def plan_pickup_object(robot, tool_frame_name, ik_joint_names, base_link_name, \
                       ik_fn, block, grasp_gen, \
                       extra_tcp_tf_fn=None,
                       fixed=[], teleport=False, enable_self_collision=True):
    """ plan trajectories for picking up the block

    The trajectory includes:
        current conf  -> approach pose
        approach pose -> pick pose
        attach object
        pick pose -> approach pose
        approach pose -> start conf

    """
    ik_grasp_fn = get_grasp_ik_fn(ik_fn, robot, ik_joint_names, base_link_name, \
                                  fixed=[], num_attempts=10,
                                  self_collisions=enable_self_collision,
                                  teleport=teleport)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), \
                                         teleport=teleport, \
                                         self_collisions=enable_self_collision)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, \
                                               teleport=teleport, \
                                               self_collisions=enable_self_collision)

    ik_joints = joints_from_names(robot, ik_joint_names)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot, joints=ik_joints)
    saved_world = WorldSaver()
    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_grasp_fn(block, pose0, grasp, extra_tcp_tf_fn=extra_tcp_tf_fn)
        if result1 is None:
            continue
        conf1, path2 = result1
        pose0.assign()
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        return Command(path1.body_paths + path2.body_paths + path3.body_paths)
    return None

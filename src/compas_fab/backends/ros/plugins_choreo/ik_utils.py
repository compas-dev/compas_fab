""" ik util functions

These utility functions are modified from `ss-pybullet <https://github.com/caelan/ss-pybullet/tree/master/pybullet_tools/ikfast>`. The reason for this adaption is to give `compas_fab`
users more control on using IKfast,

For now, these functions take only pybullet-based parameters,
e.g. Pose = (point, quat). But these will later be converted to compas objects.

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from conrob_pybullet import multiply, get_link_pose, get_movable_joints, \
    link_from_name, get_joint_positions, invert, violates_limits, get_pose, \
    get_distance, joints_from_names, quat_from_matrix

# TODO: default options, should be removed later
USE_ALL = False
USE_CURRENT = None


def get_ik_tool_link_pose(fk_fn, robot, ik_joint_names, base_link_name, \
                          joint_values=None, use_current=USE_CURRENT):
    """Use the given forward_kinematics function to compute ik_tool_link pose
    based on current joint configurations in pybullet.

    The resulting FK pose is relative to the world frame, not the robot's base frame.

    Parameters
    ----------
    fk_fn : function handle
        fk(a 6-list) : point, rot_matrix
    robot : pybullet robot
    ik_joint_names : list of str
        a list of joint names that is registered for IK/FK
    base_link_name : str
        robot base link's name, usually it's simply 'base_link'
    joint_values : list of float
        robot joint values for FK computation, value correponds to ik_joint_names

    Returns
    -------
    pybullet Pose
        Pose = (point, quat) = ([x,y,z], [4-list])

    """
    ik_joints = joints_from_names(robot, ik_joint_names)
    if use_current:
        conf = get_joint_positions(robot, ik_joints)
    else:
        assert joint_values
        conf = joint_values

    base_from_tool = compute_forward_kinematics(fk_fn, conf)
    world_from_base = get_link_pose(robot, link_from_name(robot, base_link_name))
    return multiply(world_from_base, base_from_tool)


# def get_tool_from_ik(robot):
#     world_from_tool = get_link_pose(robot, link_from_name(robot, TOOL_FRAME))
#     world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAME))
#     # tool from the bare flange (6th axis)
#     return multiply(invert(world_from_tool), world_from_ik)


def get_ik_generator(ik_fn, robot, base_link_name, world_from_tcp, ik_tool_link_from_tcp):
    world_from_base = get_link_pose(robot, link_from_name(robot, base_link_name))
    base_from_tcp = multiply(invert(world_from_base), world_from_tcp)
    base_from_ik_tool_link = multiply(base_from_tcp, invert(ik_tool_link_from_tcp))
    yield compute_inverse_kinematics(ik_fn, base_from_ik_tool_link)


def sample_tool_ik(ik_fn, robot, ik_joints, base_link_name, world_from_tcp, ik_tool_link_from_tcp, closest_only=False, get_all=False, **kwargs):
    """ sample ik joints for a given tcp pose in the world frame

    Parameters
    ----------
    ik_fn : type
        Description of parameter `ik_fn`.
    robot : type
        Description of parameter `robot`.
    ik_joints : type
        Description of parameter `ik_joints`.
    base_link_name : type
        Description of parameter `base_link_name`.
    world_from_tcp : type
        Description of parameter `world_from_tcp`.
    ik_tool_link_from_tcp : type
        Description of parameter `ik_tool_link_from_tcp`.
    closest_only : type
        Description of parameter `closest_only`.
    get_all : type
        Description of parameter `get_all`.
    **kwargs : type
        Description of parameter `**kwargs`.

    Returns
    -------
    type
        Description of returned object.

    """
    generator = get_ik_generator(ik_fn, robot, base_link_name, world_from_tcp, ik_tool_link_from_tcp)
    # ik_joints = get_movable_joints(robot)
    solutions = next(generator)
    if closest_only and solutions:
        current_conf = get_joint_positions(robot, ik_joints)
        solutions = [min(solutions, key=lambda conf: get_distance(current_conf, conf))]
    solutions = list(filter(lambda conf: not violates_limits(robot, ik_joints, conf), solutions))
    return solutions if get_all else select_solution(robot, ik_joints, solutions, **kwargs)


def compute_forward_kinematics(fk_fn, conf):
    pose = fk_fn(list(conf))
    pos, rot = pose
    quat = quat_from_matrix(rot) # [X,Y,Z,W]
    return pos, quat


def compute_inverse_kinematics(ik_fn, pose, sampled=[]):
    pos = point_from_pose(pose)
    rot = matrix_from_quat(quat_from_pose(pose)).tolist()
    if sampled:
        solutions = ik_fn(list(rot), list(pos), sampled)
    else:
        solutions = ik_fn(list(rot), list(pos))
    if solutions is None:
        return []
    return solutions


def get_ik_limits(robot, joint, limits=USE_ALL):
    if limits is USE_ALL:
        return get_joint_limits(robot, joint)
    elif limits is USE_CURRENT:
        value = get_joint_position(robot, joint)
        return value, value
    return limits


def select_solution(body, joints, solutions, nearby_conf=USE_ALL, **kwargs):
    if not solutions:
        return None
    if nearby_conf is USE_ALL:
        return random.choice(solutions)
    if nearby_conf is USE_CURRENT:
        nearby_conf = get_joint_positions(body, joints)
    # TODO: sort by distance before collision checking
    # TODO: search over neighborhood of sampled joints when nearby_conf != None
    return min(solutions, key=lambda conf: get_distance(nearby_conf, conf, **kwargs))

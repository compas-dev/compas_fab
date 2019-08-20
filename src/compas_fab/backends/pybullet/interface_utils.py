"""Interface utililty functions for connecting compas_fab and pybullet.

Note: Now these functions are mainly used for connecting compas_fab and pychoreo.

"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
from tempfile import TemporaryDirectory

from compas.geometry import Frame, Transformation

from conrob_pybullet import Pose
from conrob_pybullet import create_obj, set_pose, quat_from_matrix, add_body_name, \
    quat_from_euler, link_from_name, get_link_pose, add_fixed_constraint, euler_from_quat, \
    multiply, is_connected, load_pybullet, joints_from_names, set_joint_positions, \
    get_constraint_info, create_attachment, get_pose, pairwise_collision, set_color, \
    wait_for_user, has_link

# TODO: this will be added later
# __all__ = [
#            ''
# ]

def generate_rel_path_URDF_pkg(input_urdf_path, pkg_name):
    """generate a URDF file with relative linking paths in the input file's dir

    Note: this util function is a temporal workaround for the unimplemented
    compas.robots.Robot.generate_robot_package function, which should be able
    to generate a ros pkg for the robot description, with an optional argument
    to use relative path or ros-package path.

    For example, the original URDF file has the following paths:

            <visual>
                <geometry>
                    <mesh filename="$(find pkg_name)/meshes/visual/xxx.stl" scale="1 1 1" />
                </geometry>
            </visual>

    This function modifies it to the following form, and keeps everything else
    untouched:

            <visual>
                <geometry>
                    <mesh filename="../meshes/visual/xxx.stl" scale="1 1 1" />
                </geometry>
            </visual>

    Parameters
    ----------
    input_urdf_path : str
        absolute path to the original URDF file
    pkg_name : str
        name of the ros package, e.g. 'ur_description'

    Returns
    -------
    rel_urdf_path : str
        absolute path for the generated URDF file

    """
    with open(input_urdf_path, 'r') as file :
        filedata = file.read()

    # Replace the target string
    package_prefix = 'package://' + pkg_name
    # TODO: raise if prefix not found
    filedata = filedata.replace(package_prefix, '..')

    # new file name and path
    orig_file_name = input_urdf_path.split(os.sep)[-1]
    orig_file_dir = os.path.dirname(input_urdf_path)
    urdf_rel_name = orig_file_name.split('.urdf')[0] + '_rel_link' + '.urdf'
    urdf_rel_path = os.path.join(orig_file_dir, urdf_rel_name)

    # Write the file out again
    with open(urdf_rel_path, 'w+') as file:
        file.write(filedata)

    return urdf_rel_path


def pb_pose_from_Frame(frame):
    """ convert compas.Frame to (point, quat).

    Parameters
    ----------
    frame : compas Frame

    Returns
    -------
    tuple
        (point, euler),
        where point: [x, y, z]
              quat:  [roll, pitch, yaw]

    """
    point = [frame.point.x, frame.point.y, frame.point.z]
    # zaxis = frame.xaxis.cross(frame.yaxis)
    # mat = [list(frame.xaxis), list(frame.xaxis), list(zaxis)]
    # quat = list(quat_from_matrix(mat))
    euler = frame.euler_angles()
    return (point, quat_from_euler(euler))


def pb_pose_from_Transformation(tf):
    """ convert compas.Transformation to (point, quat).

    Parameters
    ----------
    tf : compas Transformation

    Returns
    -------
    tuple
        (point, euler),
        where point: [x, y, z]
              quat:  [roll, pitch, yaw]

    """
    point = [tf[i, 3] for i in range(3)]
    quat = quat_from_matrix([tf[i, :3] for i in range(3)])
    return (point, quat)


def Frame_from_pb_pose(pose):
    """ convert (point, quat) to compas.Frame

    Parameters
    ----------
    pose : ([x,y,z], [roll, pitch, yaw])

    Returns
    -------
    frame : compas Frame

    """
    frame = Frame()
    frame.from_euler_angles(euler_from_quat(pose[1]))
    frame.point = pose[0]
    return frame


def convert_mesh_to_pybullet_body(mesh, frame=Frame.worldXY(), name=None, scale=1.0):
    """ convert compas mesh and its frame to a pybullet body

    Parameters
    ----------
    mesh : compas Mesh
    frame : compas Frame
    name : str
        Optional, name of the mesh for tagging in pybullet's GUI

    Returns
    -------
    pybullet body

    """
    # for now, py3.2+ only:
    # https://stackoverflow.com/questions/6884991/how-to-delete-dir-created-by-python-tempfile-mkdtemp
    assert is_connected()
    with TemporaryDirectory() as temp_dir:
        # print('temp_dir: {}'.format(temp_dir))
        tmp_obj_path = os.path.join(temp_dir, 'compas_mesh_temp.obj')
        mesh.to_obj(tmp_obj_path)
        pyb_body = create_obj(tmp_obj_path, scale=scale)
        body_pose = pb_pose_from_Frame(frame)
        set_pose(pyb_body, body_pose)
        if name:
            # this is just adding a tag on the GUI
            # its name might be different to the planning scene name...
            add_body_name(pyb_body, name)
    return pyb_body


def convert_meshes_and_poses_to_pybullet_bodies(co_dict, scale=1.0):
    """Convert collision mesh/pose dict fetched from compas_fab client to
    a pybullet body dict, and add them to the pybullet env

    Parameters
    ----------
    co_dict : dict
        {object_id : {'meshes' : [compas.Mesh],
                      'mesh_poses' : [compas.Frame]}}
    scale : float
        unit scale conversion to meter, default to 1.0

    Returns
    -------
    dict of pybullet bodies
        {object_id : [pybullet_body, ]}

    """
    body_dict = {}
    for name, item_dict in co_dict.items():
        n_obj = len(item_dict['meshes'])
        body_dict[name] = []
        for i, mesh, frame in zip(range(n_obj), item_dict['meshes'], item_dict['mesh_poses']):
            body_name = name + str(i) if len(item_dict['meshes']) > 1 else name
            body = convert_mesh_to_pybullet_body(mesh, frame, body_name)
            body_dict[name].append(body)
    return body_dict


def attach_end_effector_geometry(ee_meshes, robot, ee_link_name, scale=1.0):
    """ create and attach a list of end effector meshes to the robot's ee_link.

    Note: for now, only collision mesh is supported, no virual mesh.

    Parameters
    ----------
    ee_meshes : list of compas.Mesh
        Should be a list of meshes from the convex decomposition of the end effector geometry
    robot : pybullet robot
    ee_link_name : str
        name of the link where the end effector is attached to, usually is `ee_link`
    scale: float
        unit conversion factor to meter, default 1.0. For example, if the model
        is in millimeter, set it to 1e-3.

    Returns
    -------
    list of pybullet bodies
        a list of pybullet object for the ee_meshes

    """
    pyb_ee_link = link_from_name(robot, ee_link_name)
    ee_link_pose = get_link_pose(robot, pyb_ee_link)
    ee_attachs = []
    for mesh in ee_meshes:
        ee_body = convert_mesh_to_pybullet_body(mesh, Frame.worldXY())
        set_pose(ee_body, ee_link_pose)
        # constr = add_fixed_constraint(ee_body, robot, pyb_ee_link)
        # print(get_constraint_info(constr))
        attach = create_attachment(robot, pyb_ee_link, ee_body)
        ee_attachs.append(attach)
    return ee_attachs


def get_TCP_pose(robot, ee_link_name, ee_link_from_TCP_tf=None, return_pb_pose=False):
    """ get current TCP pose in pybullet, based on its current configuration state

    Parameters
    ----------
    robot : pybullet robot
    ee_link_name : str
        name of the link where the end effector is attached to, usually is `ee_link`
    ee_link_from_TCP_tf : compas Transformation
        ee_link to TCP transformation, if any, default to None
    return_pb_pose : bool
        if set True, return pb pose. Otherwise return compas Frame

    Returns
    -------
    TCP_frame : compas Frame or (point, quat)
        in the world coordinate

    """
    pyb_ee_link = link_from_name(robot, ee_link_name)
    world_from_ee_link = get_link_pose(robot, pyb_ee_link)
    if ee_link_from_TCP_tf:
        ee_link_from_TCP = pb_pose_from_Transformation(ee_link_from_TCP_tf)
        world_from_TCP = multiply(world_from_ee_link, ee_link_from_TCP)
    else:
        world_from_TCP = world_from_ee_link
    if not return_pb_pose:
        return Frame_from_pb_pose(world_from_TCP)
    else:
        return world_from_TCP


def create_pb_robot_from_ros_urdf(urdf_path, pkg_name, planning_scene=None, ee_link_name=None,
                                  keep_temp_urdf=False):
    """Create pybullet robot from ros package-based urdf.

    TODO: Ideally, the input would be a compas_fab.robots.Robot class, we generate
    the urdf from there. But, this will require some new functions in the compas.xml...

    Parameters
    ----------
    urdf_path : str
        absolute path to the ros urdf file
    pkg_name : str
        name of the ros package where the urdf resides, e.g. 'ur_description' or
        'abb_irb4600_support'
    keep_temp_urdf : bool
        Defaults to False, the generated temporal urdf where all the mesh links
        are changed to relative path, is deleted after the py_robot is created.
        Otherwise, the file is kept at the `urdf_path`.

    Returns
    -------
    type
        Description of returned object.

    """
    assert is_connected()
    try:
        rel_urdf_path = generate_rel_path_URDF_pkg(urdf_path, pkg_name)
        pb_robot = load_pybullet(rel_urdf_path, fixed_base=True)
        if planning_scene:
            # update joint conf according to the planning scene
            jt_state = planning_scene.get_joint_state()
            pb_ik_joints = joints_from_names(pb_robot, jt_state.keys())
            set_joint_positions(pb_robot, pb_ik_joints, jt_state.values())

        ee_attached_bodies = []
        # BUG: id is not in aco!
        # if planning_scene and ee_link_name:
        #     # update attached object according to the planning scene
        #     aco_dict = planning_scene.get_attached_collision_objects()
        #     for aco_name, aco in aco_dict.items():
        #         # print('{}: {} {}'.format(aco_name, aco['meshes'], aco['link_name']))
        #         ee_attached_bodies.extend(
        #             attach_end_effector_geometry(aco['meshes'], pb_robot, aco['link_name']))
    except:
        if os.path.exists(rel_urdf_path):
            os.remove(rel_urdf_path)
        assert False, 'pybullet create robot from ros urdf fails.'
    if not keep_temp_urdf:
        if os.path.exists(rel_urdf_path):
            os.remove(rel_urdf_path)
    if ee_attached_bodies:
        return pb_robot, ee_attached_bodies
    else:
        return pb_robot

def sanity_check_collisions(brick_from_index, obstacle_from_name):
    in_collision = False
    init_pose = None
    for brick in brick_from_index.values():
        for e_body in brick.pybullet_bodies:
            if not init_pose:
                init_pose = get_pose(e_body)
            for so_id, so in obstacle_from_name.items():
                set_pose(e_body, pb_pose_from_Frame(brick.initial_frame))
                if pairwise_collision(e_body, so):
                    set_color(e_body, (1, 0, 0, 0.6))
                    set_color(so, (0, 0, 1, 0.6))

                    in_collision = True
                    print('collision detected between brick #{} and static #{} in its pick pose'.format(brick.name, so_id))
                    wait_for_user()

                set_pose(e_body, pb_pose_from_Frame(brick.goal_frame))
                if pairwise_collision(e_body, so):
                    in_collision = True
                    print('collision detected between brick #{} and static #{} in its place pose'.format(brick.name, so_id))
                    wait_for_user()

    # # reset their poses for visual...
    for brick in brick_from_index.values():
        for e_body in brick.pybullet_bodies:
            set_pose(e_body, init_pose)
    return in_collision

def get_pb_robot_disabled_self_collisions(pb_robot, disabled_collision_names):
    """get disabled robot link pairs for pybullet collision

    Parameters
    ----------
    pb_robot : pybullet robot
    disabled_collision_names : list of string-tuples
        DISABLED_COLLISIONS = [
            ('robot_link_4', 'workspace_objects'),
            ('robot_link_5', 'eef_base_link'),
        ]

    Returns
    -------
    [type]
        [description]
    """

    return {tuple(link_from_name(pb_robot, link) for link in pair if has_link(pb_robot, link))
                  for pair in disabled_collision_names}

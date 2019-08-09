"""Interface definition for the choreo planer.

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
from tempfile import TemporaryDirectory

from compas.geometry import Frame, Transformation
from compas_fab.backends.ros.plugins import PlannerPlugin

from conrob_pybullet import Pose
from conrob_pybullet import create_obj, set_pose, quat_from_matrix, add_body_name, \
    quat_from_euler, link_from_name, get_link_pose, add_fixed_constraint, euler_from_quat, \
    multiply, is_connected, load_pybullet

from conrob_pybullet import wait_for_user

# let's do this later...
# __all__ = [
#     'ChoreoPlanner',
#     'generate_rel_path_URDF_pkg',
#     'convert_mesh_to_pybullet_body',
#     'attach_end_effector_geometry',
#     'pb_pose_from_Frame',
#     'get_TCP_pose',
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


def convert_mesh_to_pybullet_body(mesh, frame, name=None, scale=1.0):
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
    with TemporaryDirectory() as temp_dir:
        print('temp_dir: {}'.format(temp_dir))
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
    ee_bodies = []
    for mesh in ee_meshes:
        ee_body = convert_mesh_to_pybullet_body(mesh, Frame.worldXY())
        set_pose(ee_body, ee_link_pose)
        add_fixed_constraint(ee_body, robot, pyb_ee_link)
        ee_bodies.append(ee_body)
    return ee_bodies


def get_TCP_pose(robot, ee_link_name, ee_link_from_TCP_tf, return_pb_pose=False):
    """ get current TCP pose in pybullet, based on its current configuration state

    Parameters
    ----------
    robot : pybullet robot
    ee_link_name : str
        name of the link where the end effector is attached to, usually is `ee_link`
    ee_link_from_TCP_tf : compas Transformation
        ee_link to TCP transformation
    return_pb_pose : bool
        if set True, return pb pose. Otherwise return compas Frame

    Returns
    -------
    TCP_frame : compas Frame or (point, quat)
        in the world coordinate

    """
    pyb_ee_link = link_from_name(robot, ee_link_name)
    world_from_ee_link = get_link_pose(robot, pyb_ee_link)
    ee_link_from_TCP = pb_pose_from_Transformation(ee_link_from_TCP_tf)
    world_from_TCP = multiply(world_from_ee_link, ee_link_from_TCP)
    if not return_pb_pose:
        return Frame_from_pb_pose(world_from_TCP)
    else:
        return world_from_TCP


def create_pb_robot_from_ros_urdf(urdf_path, pkg_name, keep_temp_urdf=False):
    """Create pybullet robot from ros package-based urdf.

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
    except:
        if os.path.exists(rel_urdf_path):
            os.remove(rel_urdf_path)
        assert False, 'pybullet create robot from ros urdf fails.'
    if not keep_temp_urdf:
        if os.path.exists(rel_urdf_path):
            os.remove(rel_urdf_path)
    return pb_robot


class ChoreoPlanner(PlannerPlugin):
    """Implement the planner backend interface based on choreo
    """

    def inverse_kinematics_async(self, callback, errback, frame, base_link, group,
                                 joint_names, joint_positions, avoid_collisions=True,
                                 constraints=None, attempts=8, attached_collision_meshes=None):
        raise NotImplementedError

    def forward_kinematics_async(self, callback, errback, joint_positions, base_link,
                                 group, joint_names, ee_link):
        raise NotImplementedError

    def plan_cartesian_motions_async(self, callback, errback, frames, base_link,
                                     ee_link, group, joint_names, joint_types,
                                     start_configuration, max_step, jump_threshold,
                                     avoid_collisions, path_constraints,
                                     attached_collision_meshes):
        raise NotImplementedError

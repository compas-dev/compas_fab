"""Interface definition for the choreo planer.

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
from tempfile import TemporaryDirectory

from compas_fab.backends.ros.plugins import PlannerPlugin

from conrob_pybullet import Pose
from conrob_pybullet import create_obj, set_pose, quat_from_matrix, add_body_name, \
    quat_from_euler

__all__ = [
    'ChoreoPlanner',
    'generate_rel_path_URDF_pkg',
    'convert_Frame_to_pybullet_pose',
]

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


def convert_Frame_to_pybullet_pose(frame):
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


def convert_mesh_to_pybullet_body(mesh, frame, name=None):
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
        pyb_body = create_obj(tmp_obj_path)
        body_pose = convert_Frame_to_pybullet_pose(frame)
        set_pose(pyb_body, body_pose)
        if not name:
            # this is just adding a tag on the GUI
            # its name might be different to the planning scene name...
            add_body_name(pyb_body, name)
    return pyb_body


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

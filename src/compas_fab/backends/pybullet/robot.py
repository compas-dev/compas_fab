"""

"""

import os

from pybullet_planning import WorldSaver
from pybullet_planning import is_connected, get_pose, set_pose, link_from_name, get_link_pose, has_link, \
    multiply, create_attachment


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
    assert is_connected(), 'pybullet environment not initiated'
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

##################################################

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
    from compas.geometry import Frame
    from compas_fab.backends.pybullet.body import convert_mesh_to_pybullet_body

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
    from compas_fab.backends.pybullet import pb_pose_from_Transformation, Frame_from_pb_pose

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

##################################################

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

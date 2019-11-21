"""

"""

import os

from pybullet_planning import WorldSaver
from pybullet_planning import is_connected, get_pose, set_pose, link_from_name, get_link_pose, has_link, \
    multiply, create_attachment

##################################################

def attach_end_effector_geometry(ee_meshes, robot, ee_link_name, scale=1.0):
    """ create and attach a list of end effector meshes to the robot's ee_link.

    Note: for now, only collision mesh is supported, no virual mesh.

    Parameters
    ----------
    ee_meshes : list of compas.Mesh
        Should be a list of meshes from the convex decomposition of the end effector geometry
    robot :
        index of the robot body in pybullet
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

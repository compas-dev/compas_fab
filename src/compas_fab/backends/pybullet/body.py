
from compas.geometry import Frame
from pybullet_planning import create_obj, is_connected, set_pose, add_body_name

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
    import os, sys
    if sys.version_info[0] < 3:
        from backports import tempfile
    else:
        import tempfile
    from compas_fab.backends.pybullet.pose import pb_pose_from_Frame

    assert is_connected(), 'pybullet env not initiated'
    with tempfile.TemporaryDirectory() as temp_dir:
        tmp_obj_path = os.path.join(temp_dir, 'compas_mesh_temp.obj')
        mesh.to_obj(tmp_obj_path)
        pyb_body = create_obj(tmp_obj_path, scale=scale)
        body_pose = pb_pose_from_Frame(frame)
        set_pose(pyb_body, body_pose)
        if name:
            # this is just adding a tag on the GUI
            # its name might be different to the planning scene name
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

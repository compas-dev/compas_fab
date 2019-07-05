from collections import OrderedDict
from compas.geometry import Transformation, transform_points, cross_vectors

ELEMENT_KEY_PREFIX = 'e'
VIRTUAL_JOINT_KEY_PREFIX = 'vj'
KEY_SEPARATOR = '_'

def element_vert_key(id):
    return ELEMENT_KEY_PREFIX + KEY_SEPARATOR + str(id)

def extract_element_vert_id(str_key):
    key_sep = str_key.split(KEY_SEPARATOR)
    if key_sep[0] == ELEMENT_KEY_PREFIX:
        return int(key_sep[1])
    else:
        return None

def virtual_joint_key(id):
    return VIRTUAL_JOINT_KEY_PREFIX + KEY_SEPARATOR + str(id)

def extract_virtual_joint_vert_id(str_key):
    return None

def obj_name(obj_key, sub_mesh_id):
    return obj_key + '_' + str(sub_mesh_id) + '.obj'

# -----------------
# json export utils
# -----------------
def cframe2json(cfr):
    """convert compas frame into a json dict

    Parameters
    ----------
    cfr : compas frame

    Returns
    -------
    dict
        a dict that is compatible with choreo

    """
    dict = OrderedDict()
    dict["XAxis"] = cfr.to_data()["xaxis"]
    dict["YAxis"] = cfr.to_data()["yaxis"]
    dict["ZAxis"] = cross_vectors(dict["XAxis"], dict["YAxis"])
    dict["Origin"] = cfr.to_data()["point"]
    return dict

def transform_cmesh(cmesh, ctf):
    """transform a compas mesh
    Paramters
    ---------
    cmesh: a compas mesh
    ctf: a compas transformation
    """
    verts, faces = cmesh.to_vertices_and_faces()
    new_mesh = cmesh.copy()
    for i, v in enumerate(verts):
        v_tf = transform_points([v], ctf)[0]
        new_mesh.set_vertex_attributes(i, ['x', 'y', 'z'], v_tf)
    return new_mesh

def transform_cmesh_to_origin(cmesh, mesh_pose_in_scene):
    obj_tf = Transformation.from_frame(mesh_pose_in_scene)
    return transform_cmesh(cmesh, obj_tf.inverse())

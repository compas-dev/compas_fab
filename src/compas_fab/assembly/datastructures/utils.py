from collections import OrderedDict
from compas.geometry import Transformation, transform_points, cross_vectors


def element_vert_key(id):
    return 'e_' + str(id)

def extract_element_vert_id(str_key):
    pass

def virtual_joint_key(id):
    return 'vj_' + str(id)

def extract_virtual_joint_vert_id(str_key):
    pass

# -----------------
# json export utils
# -----------------
def obj_name(obj_id, sub_id):
    # TODO: just for now...
    name = "object_" + str(obj_id) + "-" + str(sub_id) + ".obj"
    return name

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
    for i, v in enumerate(verts):
        v_tf = transform_points([v], ctf)[0]
        cmesh.set_vertex_attributes(i, ['x', 'y', 'z'], v_tf)
    return cmesh

def transform_cmesh_to_origin(cmesh, mesh_pose_in_scene):
    obj_tf = Transformation.from_frame(mesh_pose_in_scene)
    return transform_cmesh(cmesh, obj_tf.inverse())

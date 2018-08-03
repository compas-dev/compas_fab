# Support pre-release versions of compas
try:
    from compas.geometry.basic import multiply_matrices
except ImportError:
    from compas.geometry.utilities import multiply_matrices

from compas.geometry import transform

# TODO: must move somewhere into compas core

def mesh_update_vertices(mesh, vertices):
    for i in range(len(vertices)):
        mesh.vertex[i].update({'x': vertices[i][0], 'y': vertices[i][1], 'z': vertices[i][2]})


def mesh_transform(mesh, transformation, copy=True):
    if copy:
        m = mesh.copy()
    else:
        m = mesh
    xyz = transform(m.xyz, transformation.matrix)
    mesh_update_vertices(m, xyz)
    return m

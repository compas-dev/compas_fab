from compas.geometry.basic import multiply_matrices

# TODO: move this somewhere else !!! to mesh algorithms or operations ??
def mesh_get_transformed_vertices(mesh, transformation_matrix):
    xyz = zip(*mesh.xyz) # transpose matrix
    xyz += [[1] * len(xyz[0])] # homogenize
    xyz = multiply_matrices(transformation_matrix, xyz)
    return zip(*xyz[:3])

def mesh_update_vertices(mesh, vertices):
    for i in range(len(vertices)):
        mesh.vertex[i].update({'x':vertices[i][0], 'y':vertices[i][1], 'z':vertices[i][2]})
        
def mesh_transform(mesh, transformation_matrix):
    vertices = mesh_get_transformed_vertices(mesh, transformation_matrix)
    mesh_update_vertices(mesh, vertices)
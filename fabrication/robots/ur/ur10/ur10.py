'''
Created on 15.06.2017

@author: rustr
'''

from compas_fabrication.fabrication.robots.ur import UR
#from compas_fabrication.fabrication.grasshopper.mesh import draw_mesh, draw_line, mesh_transform
from compas_fabrication.fabrication.geometry import Frame, Rotation
from compas.geometry.elements import Line, Point
from compas.datastructures.mesh import Mesh
from compas.geometry.utilities import multiply_matrices

import os
import time

def mesh_transform(mesh, transformation_matrix, copy=False):
        
    xyz = zip(*mesh.xyz) # transpose matrix
    xyz += [[1] * len(xyz[0])] # homogenize
    xyz = multiply_matrices(transformation_matrix, xyz)
    
    if copy:
        mesh = mesh.copy()
    for i in range(len(xyz[0])):
        mesh.vertex[i].update({'x':xyz[0][i], 'y':xyz[1][i], 'z':xyz[2][i]})
    return mesh


def mesh_get_transformed_vertices(mesh, transformation_matrix):
    xyz = zip(*mesh.xyz) # transpose matrix
    xyz += [[1] * len(xyz[0])] # homogenize
    xyz = multiply_matrices(transformation_matrix, xyz)
    return zip(*xyz[:3])


class UR10(UR):
    """
    #define UR10_PARAMS
    https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    but in mm, not in m
    """
    
    #define UR10_PARAMS
    d1 = 127.3
    a2 = -612.0
    a3 = -572.3
    d4 = 163.941
    d5 = 115.7
    d6 = 92.2
    
    shoulder_offset = 220.941
    elbow_offset = -171.9
    
    working_area_diameter = 2650
    # recommended Reach diameter 2600
    # Max. working area 2650
    # sphere with radius, max working area and center of j0end, cutoff cylinder : diameter 190
    
    # the axes around which the joints get rotated
    # m0 rotates arount j0, m1 around j1, ...
    j0 = Line((0, 0, 0),                 (0, 0, d1))
    j1 = Line((0, 0, d1),                (0, -shoulder_offset, d1))
    j2 = Line((a2, -shoulder_offset-elbow_offset, d1), (a2, -shoulder_offset, d1))
    j3 = Line((a2+a3, 0, d1),            (a2+a3,-d4, d1))
    j4 = Line((a2+a3, -d4, d1),          (a2+a3, -d4, d1-d5))
    j5 = Line((a2+a3, -d4, d1-d5),       (a2+a3, -d4-d6, d1-d5))
    
    
    # the joints loaded as meshes
    path = os.path.join(os.path.dirname(__file__), "model")
    m0 = Mesh.from_obj(os.path.join(path, 'base_and_shoulder.obj'))
    m1 = Mesh.from_obj(os.path.join(path, 'upperarm.obj'))
    m2 = Mesh.from_obj(os.path.join(path, 'forearm.obj'))
    m3 = Mesh.from_obj(os.path.join(path, 'wrist1.obj'))
    m4 = Mesh.from_obj(os.path.join(path, 'wrist2.obj'))
    m5 = Mesh.from_obj(os.path.join(path, 'wrist3.obj'))
    
    m0_faces = [m0.face_vertices(fkey, True) for fkey in m0.faces_iter()]
    m1_faces = [m1.face_vertices(fkey, True) for fkey in m1.faces_iter()]
    m2_faces = [m2.face_vertices(fkey, True) for fkey in m2.faces_iter()]
    m3_faces = [m3.face_vertices(fkey, True) for fkey in m3.faces_iter()]
    m4_faces = [m4.face_vertices(fkey, True) for fkey in m4.faces_iter()]
    m5_faces = [m5.face_vertices(fkey, True) for fkey in m5.faces_iter()]
    
    tool0_frame = Frame(j5[1], [1,0,0], [0,0,1])
    tool_frame = Frame(j5[1], [1,0,0], [0,0,1])
        
    def __init__(self):
        pass
    
    def get_forward_transformations(self, q):
        
        q0, q1, q2, q3, q4, q5 = q
        j0, j1, j2, j3, j4, j5 = self.j0, self.j1, self.j2, self.j3, self.j4, self.j5

        R0 = Rotation.from_axis_and_angle(j0.vector, q0, j0.end)
        j1 = Line(R0 * j1.start, R0 * j1.end)
        R1 = Rotation.from_axis_and_angle(j1.vector, q1, j1.end) * R0
        j2 = Line(R1 * j2.start, R1 * j2.end)
        R2 = Rotation.from_axis_and_angle(j2.vector, q2, j2.end) * R1
        j3 = Line(R2 * j3.start, R2 * j3.end)
        R3 = Rotation.from_axis_and_angle(j3.vector, q3, j3.end) * R2
        j4 = Line(R3 * j4.start, R3 * j4.end)
        R4 = Rotation.from_axis_and_angle(j4.vector, q4, j4.end) * R3
        j5 = Line(R4 * j5.start, R4 * j5.end)
        R5 = Rotation.from_axis_and_angle(j5.vector, q5, j5.end) * R4
        
        return R0, R1, R2, R3, R4, R5
    
    def get_transformed_model(self, q):
        
        R0, R1, R2, R3, R4, R5 = self.get_forward_transformations(q)
        
        """
        m0_xyz = mesh_get_transformed_vertices(self.m0, R0.matrix)
        m1_xyz = mesh_get_transformed_vertices(self.m1, R1.matrix)
        m2_xyz = mesh_get_transformed_vertices(self.m2, R2.matrix)
        m3_xyz = mesh_get_transformed_vertices(self.m3, R3.matrix)
        m4_xyz = mesh_get_transformed_vertices(self.m4, R4.matrix)
        m5_xyz = mesh_get_transformed_vertices(self.m5, R5.matrix)
        """
        m0_xyz = R0 * self.m0.xyz
        m1_xyz = R1 * self.m1.xyz
        m2_xyz = R2 * self.m2.xyz
        m3_xyz = R3 * self.m3.xyz
        m4_xyz = R4 * self.m4.xyz
        m5_xyz = R5 * self.m5.xyz
                
        m0 = Mesh.from_vertices_and_faces(m0_xyz, self.m0_faces)
        m1 = Mesh.from_vertices_and_faces(m1_xyz, self.m1_faces)
        m2 = Mesh.from_vertices_and_faces(m2_xyz, self.m2_faces)
        m3 = Mesh.from_vertices_and_faces(m3_xyz, self.m3_faces)
        m4 = Mesh.from_vertices_and_faces(m4_xyz, self.m4_faces)
        m5 = Mesh.from_vertices_and_faces(m5_xyz, self.m5_faces)
        
        tool0_frame = self.tool0_frame.transform(R5, copy=True)
        tool_frame = self.tool_frame.transform(R5, copy=True)
        
        return m0, m1, m2, m3, m4, m5, tool0_frame, tool_frame   

def main():
    ur10 = UR10()
    q = [-1.225745, -2.024058, 1.936111, -1.756499, 4.980559, 1.553081]
    ur10.get_transformed_model(q)

if __name__ == "__main__":
    import cProfile
    cProfile.run('main()')
    #main()
    
        
        
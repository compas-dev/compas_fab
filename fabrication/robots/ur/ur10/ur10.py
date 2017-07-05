'''
Created on 15.06.2017

@author: rustr
'''

from compas_fabrication.fabrication.robots.ur import UR
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

def update_mesh_vertices(mesh, vertices):
    for i in range(len(vertices)):
        mesh.vertex[i].update({'x':vertices[i][0], 'y':vertices[i][1], 'z':vertices[i][2]})

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
    
    # j0 - j5 are the axes around which the joints m0 - m5 rotate, e.g. m0 
    # rotates around j0, m1 around j1, etc.
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
    
    # have a copy of the not transformed vertices
    m0_xyz = m0.xyz
    m1_xyz = m1.xyz
    m2_xyz = m2.xyz
    m3_xyz = m3.xyz
    m4_xyz = m4.xyz
    m5_xyz = m5.xyz
    
    tool0_frame = Frame(j5[1], [1,0,0], [0,0,1])
    tool_frame = Frame(j5[1], [1,0,0], [0,0,1])
        
    def __init__(self):
        pass
    
    def get_forward_transformations(self, q):
        
        q0, q1, q2, q3, q4, q5 = q
        j0, j1, j2, j3, j4, j5 = self.j0, self.j1, self.j2, self.j3, self.j4, self.j5

        R0 = Rotation.from_axis_and_angle(j0.vector, q0, j0.end)
        j1 = Line(R0.transform(j1.start), R0.transform(j1.end))
        R1 = Rotation.from_axis_and_angle(j1.vector, q1, j1.end) * R0
        j2 = Line(R1.transform(j2.start), R1.transform(j2.end))
        R2 = Rotation.from_axis_and_angle(j2.vector, q2, j2.end) * R1
        j3 = Line(R2.transform(j3.start), R2.transform(j3.end))
        R3 = Rotation.from_axis_and_angle(j3.vector, q3, j3.end) * R2
        j4 = Line(R3.transform(j4.start), R3.transform(j4.end))
        R4 = Rotation.from_axis_and_angle(j4.vector, q4, j4.end) * R3
        j5 = Line(R4.transform(j5.start), R4.transform(j5.end))
        R5 = Rotation.from_axis_and_angle(j5.vector, q5, j5.end) * R4
        
        return R0, R1, R2, R3, R4, R5
    
    def get_transformed_model(self, q):
        """
        Get the transformed meshes of the robot model.
        """
        
        R0, R1, R2, R3, R4, R5 = self.get_forward_transformations(q)
        
        # transform the original vertices, otherwise the already transformed
        # vertices are transformed
        m0_xyz = R0.transform(self.m0_xyz)
        m1_xyz = R1.transform(self.m1_xyz)
        m2_xyz = R2.transform(self.m2_xyz)
        m3_xyz = R3.transform(self.m3_xyz)
        m4_xyz = R4.transform(self.m4_xyz)
        m5_xyz = R5.transform(self.m5_xyz)
        
        # update the meshes
        update_mesh_vertices(self.m0, m0_xyz)
        update_mesh_vertices(self.m1, m1_xyz)
        update_mesh_vertices(self.m2, m2_xyz)
        update_mesh_vertices(self.m3, m3_xyz)
        update_mesh_vertices(self.m4, m4_xyz)
        update_mesh_vertices(self.m5, m5_xyz)
                
        tool0_frame = self.tool0_frame.transform(R5)
        tool_frame = self.tool_frame.transform(R5)
        
        return self.m0, self.m1, self.m2, self.m3, self.m4, self.m5, tool0_frame, tool_frame   


def main():
    import time
    ur10 = UR10()
    q = [-1.225745, -2.024058, 1.936111, -1.756499, 4.980559, 1.553081]
    t0 = time.time()
    ur10.get_transformed_model(q)
    print time.time() - t0


if __name__ == "__main__":
    #import cProfile
    #cProfile.run('main()')
    main()
    
        
        
'''
Created on 21.06.2017

@author: rustr
'''

import os
import math
from compas.datastructures.mesh import Mesh
from compas_fab.fab.robots import Robot, BaseConfiguration
from compas_fab.fab.geometry import Frame, Rotation, Transformation
from compas_fab.fab.geometry.helpers import mesh_update_vertices
from .kinematics import forward_kinematics, inverse_kinematics

class UR(Robot):
    """The UR robot class.
    """

    d1 = 0
    a2 = 0
    a3 = 0
    d4 = 0
    d5 = 0
    d6 = 0

    shoulder_offset = 0
    elbow_offset = 0

    def __init__(self, client=None, client_options=None):
        super(UR, self).__init__(client, client_options)
        
        self.dof = 6
        self.external_axes = 0
        self.config_cls = BaseConfiguration

        d1, a2, a3, d4, d5, d6 = self.params
        
        # j0 - j5 are the axes around which the joints m0 - m5 rotate, e.g. m0 
        # rotates around j0, m1 around j1, etc.
        self.j0 = [(0, 0, 0),                 (0, 0, d1)]
        self.j1 = [(0, 0, d1),                (0, -self.shoulder_offset, d1)]
        self.j2 = [(a2, -self.shoulder_offset-self.elbow_offset, d1), (a2, -self.shoulder_offset, d1)]
        self.j3 = [(a2+a3, 0, d1),            (a2+a3,-d4, d1)]
        self.j4 = [(a2+a3, -d4, d1),          (a2+a3, -d4, d1-d5)]
        self.j5 = [(a2+a3, -d4, d1-d5),       (a2+a3, -d4-d6, d1-d5)]        
        
        # check difference ur5 and ur10!!!
        self.tool0_frame = Frame(self.j5[1], [1,0,0], [0,0,1])
        #self.tool0_frame = Frame(self.j5[1], [-1,0,0], [0,0,-1])
        
    @property
    def params(self):
        """Get UR specific model parameters.
        
        Returns:
            list: UR specific model parameters.
        """
        return [self.d1, self.a2, self.a3, self.d4, self.d5, self.d6]
    
    def get_model_path(self):
        pass
    
    def load_model(self):
        """Load the geometry (meshes) of the robot.
        """
        path = self.get_model_path()
        # the joints loaded as meshes
        self.m0 = Mesh.from_obj(os.path.join(path, 'base_and_shoulder.obj'))
        self.m1 = Mesh.from_obj(os.path.join(path, 'upperarm.obj'))
        self.m2 = Mesh.from_obj(os.path.join(path, 'forearm.obj'))
        self.m3 = Mesh.from_obj(os.path.join(path, 'wrist1.obj'))
        self.m4 = Mesh.from_obj(os.path.join(path, 'wrist2.obj'))
        self.m5 = Mesh.from_obj(os.path.join(path, 'wrist3.obj'))
        
        # have a copy of the mesh vertices for later transformation
        self.m0_xyz = self.m0.xyz
        self.m1_xyz = self.m1.xyz
        self.m2_xyz = self.m2.xyz
        self.m3_xyz = self.m3.xyz
        self.m4_xyz = self.m4.xyz
        self.m5_xyz = self.m5.xyz
        
    def get_forward_transformations(self, joint_angles):
        
        def vector(line):
            start, end = line
            return [end[0] - start[0], end[1] - start[1], end[2] - start[2]]
        
        q0, q1, q2, q3, q4, q5 = joint_angles
        j0, j1, j2, j3, j4, j5 = self.j0, self.j1, self.j2, self.j3, self.j4, self.j5
        
        R0 = Rotation.from_axis_and_angle(vector(j0), q0, j0[1])
        j1 = [R0.transform(j1[0]), R0.transform(j1[1])]
        R1 = Rotation.from_axis_and_angle(vector(j1), q1, j1[1]) * R0
        j2 = [R1.transform(j2[0]), R1.transform(j2[1])]
        R2 = Rotation.from_axis_and_angle(vector(j2), q2, j2[1]) * R1
        j3 = [R2.transform(j3[0]), R2.transform(j3[1])]
        R3 = Rotation.from_axis_and_angle(vector(j3), q3, j3[1]) * R2
        j4 = [R3.transform(j4[0]), R3.transform(j4[1])]
        R4 = Rotation.from_axis_and_angle(vector(j4), q4, j4[1]) * R3
        j5 = [R4.transform(j5[0]), R4.transform(j5[1])]
        R5 = Rotation.from_axis_and_angle(vector(j5), q5, j5[1]) * R4
        
        # now apply the transformation to the base    
        R0 = self.transformation_RCS_WCS * R0
        R1 = self.transformation_RCS_WCS * R1
        R2 = self.transformation_RCS_WCS * R2
        R3 = self.transformation_RCS_WCS * R3
        R4 = self.transformation_RCS_WCS * R4
        R5 = self.transformation_RCS_WCS * R5
        
        return R0, R1, R2, R3, R4, R5
    
    def get_transformed_tool_frames(self, R5):
        tool0_frame = self.tool0_frame.transform(R5, copy=True)
        tcp_frame = Frame.from_transformation(Transformation.from_frame(tool0_frame) * self.transformation_tcp_tool0)
        return tool0_frame, tcp_frame
    
    def get_transformed_model(self, q):
        """Calculate robot model according to the configuration.
        
        Args:
            configuration (Configuration): the 6 joint angles in radians 
            
        Returns:    
            (frame): The tool0 frame in robot coordinate system (RCS).
        
        Get the transformed meshes of the robot model.
        """
        R0, R1, R2, R3, R4, R5 = self.get_forward_transformations(q)
                
        tool0_frame, tcp_frame = self.get_transformed_tool_frames(R5)
        
        # transform the original vertices, rather than the mesh.vertices,
        # otherwise the already transformed vertices are transformed
        m0_xyz = R0.transform(self.m0_xyz)
        m1_xyz = R1.transform(self.m1_xyz)
        m2_xyz = R2.transform(self.m2_xyz)
        m3_xyz = R3.transform(self.m3_xyz)
        m4_xyz = R4.transform(self.m4_xyz)
        m5_xyz = R5.transform(self.m5_xyz)
        
        # update the meshes
        mesh_update_vertices(self.m0, m0_xyz)
        mesh_update_vertices(self.m1, m1_xyz)
        mesh_update_vertices(self.m2, m2_xyz)
        mesh_update_vertices(self.m3, m3_xyz)
        mesh_update_vertices(self.m4, m4_xyz)
        mesh_update_vertices(self.m5, m5_xyz)
                
        return self.m0, self.m1, self.m2, self.m3, self.m4, self.m5, tool0_frame, tcp_frame
    
    def get_transformed_tool_model(self, tcp_frame):
        return self.tool.get_transformed_tool_model(tcp_frame)
        
    def forward_kinematics(self, configuration):
        """Forward kinematics function.
        
        Args:
            configuration (Configuration): the 6 joint angles in radians 
            
        Returns:    
            (frame): The tool0 frame in robot coordinate system (RCS).
        """
        
        return forward_kinematics(configuration, self.params)
    
    def inverse_kinematics(self, tool0_frame_RCS):
        """Inverse kinematics function.
        Args:
            tool0_frame_RCS (Frame): The tool0 frame to reach in robot 
                coordinate system (RCS).
            
        Returns:
            list: A list of possible configurations.                    
        """
        return inverse_kinematics(tool0_frame_RCS, self.params)

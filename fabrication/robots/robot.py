'''
Created on 01.03.2017

@author: rustr
'''

class Robot(object):
    """
    This is the base class for all robots.
    It contains:
    - its geometry: meshes / breps
    - communication: e.g. delegated by a client instance
    - workspace: brep
    - tool: the end-effector
    - origin, the plane it resides, eg worldXY
    - 
    self.joint_values = [0,0,0,0,0,0]
    self.tcp_plane = tcp_plane
    self.tool0_plane = tool0_plane
    
    # transform world to robot origin
    self.T_W_R = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, self.origin_plane)
    # transform robot to world
    self.T_R_W = rg.Transform.PlaneToPlane(self.origin_plane, rg.Plane.WorldXY)
    """
    def __init__(self):
        pass
    
    def forward(self):
        pass
    
    def inverse(self):
        pass
    
        
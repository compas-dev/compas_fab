'''
Created on 15.06.2017

@author: rustr
'''

from compas_fabrication.fabrication.robots.ur import UR


class UR5(UR):
    """
    #define UR5_PARAMS
    https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    but in mm, not in m
    """
    
    d1 =  89.159
    a2 = -425.00
    a3 = -392.25
    d4 =  109.15
    d5 =  94.65
    d6 =  82.3
    
    shoulder_offset = 135.85
    elbow_offset = -119.7
    
    # recommended Reach diameter 1700
    # Max. working area 1860,50 
    # sphere with radius, max working area and center of j0end, cutoff cylinder : diameter 149
    
    def __init__(self):
        self.model = None
        """
        The model is a reference to a mesh
        """
    
    def forward(self, q):
        UR.forward(self)
    
    def inverse(self):
        UR.inverse(self)
        
    
if __name__ == "__main__":
    
    pass
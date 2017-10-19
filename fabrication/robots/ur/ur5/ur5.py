import os
from compas_fabrication import get_data
from compas_fabrication.fabrication.robots.ur import UR

class UR5(UR):
    """ The UR 5 robot class.
    
    Manual link:
    #define UR5_PARAMS
    https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    but in mm, not in m
    """
    
    #define UR5_PARAMS
    d1 = 89.159
    a2 = -425.0
    a3 = -392.25
    d4 =  109.15
    d5 = 94.65
    d6 = 82.3
    
    shoulder_offset = 135.85
    elbow_offset =   -119.7
    
    # The UR has a very simple workspace: is is s sphere with a cylinder in the 
    # center cut off. The axis of this cylinder is j0, the diameter is defined 
    # below. For more info: UR manual.
    working_area_sphere_diameter =  1850. # max. working area diameter, recommended 1700
    working_area_cylinder_diameter = 149.
        
    def __init__(self):
        super(UR5, self).__init__()
        self.load_model()
        
    def get_model_path(self):
        return get_data("robots/ur/ur5")


    
if __name__ == "__main__":
    ur5 = UR5()
    print ur5.get_forward_transformations([1,3,4,5,1,0])

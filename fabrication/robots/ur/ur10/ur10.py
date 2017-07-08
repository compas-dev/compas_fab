'''
Created on 15.06.2017

@author: rustr
'''

from compas_fabrication.fabrication.robots.ur import UR

import os
import time


class UR10(UR):
    """ The UR 10 robot class.
    
    Manual link:
    #define UR10_PARAMS
    https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    but in mm, not in m
    """
    
    #define UR10_PARAMS
    d1 =  127.3
    a2 = -612.0
    a3 = -572.3
    d4 =  163.941
    d5 =  115.7
    d6 =   92.2
    
    shoulder_offset = 220.941
    elbow_offset =   -171.9
    
    # The UR has a very simple workspace: is is s sphere with a cylinder in the 
    # center cuff off. The axis of this cylinder is j0. For more info: UR manual.
    working_area_sphere_diameter =  2650. # max. working area diameter, recommended 2600
    working_area_cylinder_diameter = 190.
        
    def __init__(self):
        super(UR10, self).__init__()
        self.load_model()
        
    def get_model_path(self):
        return os.path.join(os.path.dirname(__file__), "model")

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
    UR
    
        
        
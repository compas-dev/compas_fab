from __future__ import print_function
from compas_fab import get_data
from compas_fab.fab.robots.ur import UR
import math

class UR5(UR):
    """ The UR 5 robot class.

    Manual link:
    #define UR5_PARAMS
    https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    but in mm, not in m
    """

    # define UR5_PARAMS
    d1 = 89.159
    a2 = -425.0
    a3 = -392.25
    d4 = 109.15
    d5 = 94.65
    d6 = 82.3

    shoulder_offset = 135.85
    elbow_offset = -119.7

    # The UR has a very simple workspace: is is s sphere with a cylinder in the
    # center cut off. The axis of this cylinder is j0, the diameter is defined
    # below. For more info: UR manual.
    working_area_sphere_diameter = 1850.  # max. working area diameter, recommended 1700
    working_area_cylinder_diameter = 149.

    def __init__(self):
        super(UR5, self).__init__()
        self.load_model()

    def get_model_path(self):
        return get_data("robots/ur/ur5")
        

if __name__ == "__main__":
    from compas_fab.fab.geometry import Frame
    ur5 = UR5()
    R0, R1, R2, R3, R4, R5 = ur5.get_forward_transformations([1,3,4,5,1,0])
    print(ur5.forward_kinematics([1,3,4,5,1,0]))
    
    print(ur5.get_transformed_tool_frames(R5))
    
    q = [-0.44244, -1.5318, 1.34588, -1.38512, -1.05009, -0.44941700000000001]
    pose = [-511.698, 76.6692, 515.311, 2.02974, 2.04409, -0.72373500000000002]
    
    frame_RCS = Frame.from_pose_axis_angle_vector(pose)
    
    print("frame from robot RCS {0}".format(frame_RCS))

    ur = UR5()
    print("frame forward kin {0}".format(ur.forward_kinematics(q)))
    R0, R1, R2, R3, R4, R5 = ur.get_forward_transformations(q)    
    print("frame from transform {0}".format(ur.get_transformed_tool_frames(R5)[0]))
    
    
    f = ur.forward_kinematics(q)
    q = ur.inverse_kinematics(f)
    
    print(q)


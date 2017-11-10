from __future__ import print_function
from compas_fab import get_data
from compas_fab.fab.robots.ur import UR
import math

class UR10(UR):
    """The UR 10 robot class.

    Manual link:
    #define UR10_PARAMS
    https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    but in mm, not in m
    """

    # define UR10_PARAMS
    d1 = 127.3
    a2 = -612.0
    a3 = -572.3
    d4 = 163.941
    d5 = 115.7
    d6 = 92.2

    shoulder_offset = 220.941
    elbow_offset = -171.9

    # The UR has a very simple workspace: is is s sphere with a cylinder in the
    # center cuff off. The axis of this cylinder is j0. For more info: UR manual.
    working_area_sphere_diameter = 2650.  # max. working area diameter, recommended 2600
    working_area_cylinder_diameter = 190.

    def __init__(self):
        super(UR10, self).__init__()
        
        self.load_model()

    def get_model_path(self):
        return get_data("robots/ur/ur10")
    
    def forward_kinematics(self, configuration):
        q = configuration[:]
        q[5] += math.pi
        return super(UR10, self).forward_kinematics(configuration)
                    
    def inverse_kinematics(self, tool0_frame_RCS):
        qsols = super(UR10, self).inverse_kinematics(tool0_frame_RCS)
        for i in range(len(qsols)):
            qsols[i][5] -= math.pi
        return qsols
    

def main():
    ur10 = UR10()
    q = [-0.4817717618752444, 2.900620189456401, 4.466606474692679, 3.6283476234151966, 1.5707963267948974, 5.194160742259934]
    q = [0, 0, 0, 0, 0, 0]
    R0, R1, R2, R3, R4, R5 = ur10.get_forward_transformations(q)
    print(ur10.forward_kinematics(q))
    tool0_frame = ur10.tool0_frame.transform(R5, copy=True)
    print(tool0_frame)


if __name__ == "__main__":
    from compas_fab.fab.geometry import Frame
    from compas_fab.fab.robots.ur.kinematics.path_calculation import smallest_joint_pose
    
    ur = UR10()
    
    q = [4.6733, -3.39529, 1.5404, -2.90962, -1.58137, 1.59137]
    pose = [-206.258, -865.946, 606.26, 0.037001, -0.044931, 1.55344]
    
    pose = [-576.673, -717.359, 419.691, -1.41669, -0.88598900000000003, 0.96527600000000002]
    q = [4.32717, -3.57284, 1.62216, -2.58119, -0.00038495899999999998, 1.45664]
    print("q: {0}".format(smallest_joint_pose(q)))
    f = Frame.from_pose_axis_angle_vector(pose)
    print("f: {0}".format(f))
    
    R0, R1, R2, R3, R4, R5 = ur.get_forward_transformations(q)    
    f = ur.get_transformed_tool_frames(R5)[0]
    print("f ?: {0}".format(f))
    
    #f = ur.forward_kinematics(q)
    #print("f 1: {0}".format(f))
    q = ur.inverse_kinematics(f)
    
    
    print()
    for x in q:
        print(smallest_joint_pose(x))
    print()
    
    f = Frame.from_pose_axis_angle_vector(pose)
    
    
    
    
    
    



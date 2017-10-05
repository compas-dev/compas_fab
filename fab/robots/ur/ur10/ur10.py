from __future__ import print_function
from compas_fab import get_data
from compas_fab.fab.robots.ur import UR


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


def main():
    ur10 = UR10()
    q = [-0.4817717618752444, 2.900620189456401, 4.466606474692679, 3.6283476234151966, 1.5707963267948974, 5.194160742259934]
    q = [0, 0, 0, 0, 0, 0]
    R0, R1, R2, R3, R4, R5 = ur10.get_forward_transformations(q)
    print(ur10.forward_kinematics(q))

    tool0_frame = ur10.tool0_frame.transform(R5, copy=True)

    print(tool0_frame)


if __name__ == "__main__":
    #import cProfile
    # cProfile.run('main()')
    main()

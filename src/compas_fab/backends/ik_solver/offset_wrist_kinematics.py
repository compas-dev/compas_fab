from compas.robots import Configuration
from .ik_offset_wrist import inverse_kinematics_offset_wrist
from .ik_offset_wrist import forward_kinematics_offset_wrist


UR10_PARAMS = {'d1': 0.1273,
               'a2': -0.612,
               'a3': -0.5723,
               'd4':  0.163941,
               'd5':  0.1157,
               'd6':  0.0922}

UR5_PARAMS = {'d1':  0.089159,
              'a2': -0.42500,
              'a3': -0.39225,
              'd4':  0.10915,
              'd5':  0.09465,
              'd6':  0.0823}

UR3_PARAMS = {'d1':  0.1519,
              'a2': -0.24365,
              'a3': -0.21325,
              'd4':  0.11235,
              'd5':  0.08535,
              'd6':  0.0819}


def joint_angles_to_configurations(A1, A2, A3, A4, A5, A6):
    return [Configuration.from_revolute_values([a1, a2, a3, a4, a5, a6]) for a1, a2, a3, a4, a5, a6 in zip(A1, A2, A3, A4, A5, A6)]


class OffsetWristKinematics(object):

    def __init__(self, params):
        self.d1 = params["d1"]
        self.a2 = params["a2"]
        self.a3 = params["a3"]
        self.d4 = params["d4"]
        self.d5 = params["d5"]
        self.d6 = params["d6"]
    
    @property
    def params(self):
        return [self.d1, self.a2, self.a3, self.d4, self.d5, self.d6]

    def forward(self, joint_values):
        return forward_kinematics_offset_wrist(joint_values, self.params)

    def inverse(self, frame_rcf):
        A1, A2, A3, A4, A5, A6 = inverse_kinematics_offset_wrist(frame_rcf, self.params)
        return joint_angles_to_configurations(A1, A2, A3, A4, A5, A6)


class UR3Kinematics(OffsetWristKinematics):
    """
    """
    def __init__(self):
        super(UR3Kinematics, self).__init__(UR3_PARAMS)


class UR5Kinematics(OffsetWristKinematics):
    """
    """
    def __init__(self):
        super(UR5Kinematics, self).__init__(UR5_PARAMS)


class UR10Kinematics(OffsetWristKinematics):
    """
    """
    def __init__(self):
        super(UR10Kinematics, self).__init__(UR10_PARAMS)


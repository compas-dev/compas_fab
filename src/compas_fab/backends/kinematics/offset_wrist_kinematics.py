from compas.robots import Configuration
from .offset_wrist import inverse_kinematics_offset_wrist
from .offset_wrist import forward_kinematics_offset_wrist


def joint_angles_to_configurations(A1, A2, A3, A4, A5, A6):
    return [Configuration.from_revolute_values([a1, a2, a3, a4, a5, a6]) for a1, a2, a3, a4, a5, a6 in zip(A1, A2, A3, A4, A5, A6)]


class OffsetWristKinematics(object):

    def __init__(self, params):
        self.params = params

    def forward(self, joint_values):
        return forward_kinematics_offset_wrist(joint_values, self.params)

    def inverse(self, frame_rcf):
        A1, A2, A3, A4, A5, A6 = inverse_kinematics_offset_wrist(frame_rcf, self.params)
        return joint_angles_to_configurations(A1, A2, A3, A4, A5, A6)



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

class UR3_Kinematics(OffsetWristKinematics):
    """
    """
    def __init__(self):
        params = [UR3_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR3_Kinematics, self).__init__(params)


class UR5_Kinematics(OffsetWristKinematics):
    """
    """
    def __init__(self):
        params = [UR5_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR5_Kinematics, self).__init__(params)


class UR10_Kinematics(OffsetWristKinematics):
    """
    """
    def __init__(self):
        params = [UR10_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR10_Kinematics, self).__init__(params)


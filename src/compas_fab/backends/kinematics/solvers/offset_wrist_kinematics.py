from .offset_wrist import inverse_kinematics_offset_wrist
from .offset_wrist import forward_kinematics_offset_wrist

# The following parameters for UR robots are taken from the following website:
# https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

UR10_PARAMS = {'d1': 0.1273,
               'a2': -0.612,
               'a3': -0.5723,
               'd4':  0.163941,
               'd5':  0.1157,
               'd6':  0.0922}

UR10e_PARAMS = {'d1': 0.1807,
                'a2': -0.6127,
                'a3': -0.57155,
                'd4':  0.17415,
                'd5':  0.11985,
                'd6':  0.11655}

UR5_PARAMS = {'d1':  0.089159,
              'a2': -0.42500,
              'a3': -0.39225,
              'd4':  0.10915,
              'd5':  0.09465,
              'd6':  0.0823}

UR5e_PARAMS = {'d1':  0.1625,
               'a2': -0.425,
               'a3': -0.3922,
               'd4':  0.1333,
               'd5':  0.0997,
               'd6':  0.0996}


UR3_PARAMS = {'d1':  0.1519,
              'a2': -0.24365,
              'a3': -0.21325,
              'd4':  0.11235,
              'd5':  0.08535,
              'd6':  0.0819}

UR3e_PARAMS = {'d1': 0.15185,
               'a2': -0.24355,
               'a3': -0.2132,
               'd4':  0.13105,
               'd5':  0.08535,
               'd6':  0.0921}


class OffsetWristKinematics(object):
    """
    """

    def __init__(self, params):
        self.params = params

    def forward(self, joint_values):
        return forward_kinematics_offset_wrist(joint_values, self.params)

    def inverse(self, frame_rcf):
        return inverse_kinematics_offset_wrist(frame_rcf, self.params)


class UR3Kinematics(OffsetWristKinematics):
    """Analytical IK solver for UR3 robots."""
    def __init__(self):
        params = [UR3_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR3Kinematics, self).__init__(params)


class UR3eKinematics(OffsetWristKinematics):
    """Analytical IK solver for UR3 e-Series robots."""
    def __init__(self):
        params = [UR3_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR3eKinematics, self).__init__(params)


class UR5Kinematics(OffsetWristKinematics):
    """Analytical IK solver for UR5 robots."""
    def __init__(self):
        params = [UR5_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR5Kinematics, self).__init__(params)


class UR5eKinematics(OffsetWristKinematics):
    """Analytical IK solver for UR5 e-Series robots."""
    def __init__(self):
        params = [UR5e_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR5eKinematics, self).__init__(params)


class UR10Kinematics(OffsetWristKinematics):
    """Analytical IK solver for UR10 e-Series robots."""
    def __init__(self):
        params = [UR10_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR10Kinematics, self).__init__(params)


class UR10eKinematics(OffsetWristKinematics):
    """Analytical IK solver for UR10 e-Series robots."""
    def __init__(self):
        params = [UR10_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR10eKinematics, self).__init__(params)

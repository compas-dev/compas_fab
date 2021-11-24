from .offset_wrist import inverse_kinematics_offset_wrist
from .offset_wrist import forward_kinematics_offset_wrist


class OffsetWristKinematics(object):
    """
    """

    def __init__(self, params):
        self.params = params

    def forward(self, joint_values):
        return forward_kinematics_offset_wrist(joint_values, self.params)

    def inverse(self, frame_rcf):
        return inverse_kinematics_offset_wrist(frame_rcf, self.params)


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

# https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
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


class UR3(OffsetWristKinematics):
    """
    """

    def __init__(self):
        params = [UR3_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR3, self).__init__(params)


class UR5(OffsetWristKinematics):
    """
    """

    def __init__(self):
        params = [UR5_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR5, self).__init__(params)


class UR5e(OffsetWristKinematics):
    """
    """

    def __init__(self):
        params = [UR5e_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR5e, self).__init__(params)


class UR10(OffsetWristKinematics):
    """
    """

    def __init__(self):
        params = [UR10_PARAMS[k] for k in ['d1', 'a2', 'a3', 'd4', 'd5', 'd6']]
        super(UR10, self).__init__(params)


if __name__ == "__main__":
    from compas.geometry import allclose

    kin = UR3()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))

    kin = UR5()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))

    kin = UR10()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))

    kin = UR5e()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))

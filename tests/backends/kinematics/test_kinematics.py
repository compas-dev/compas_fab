from compas.geometry import allclose

from compas_fab.backends.kinematics.offset_wrist_kinematics import UR5eKinematics

def test_kinematic_functions():
    kin = UR5eKinematics()
    q = [0.2, 0.5, 1.4, 1.3, 2.6, 2.3]
    frame = kin.forward(q)
    sol = kin.inverse(frame)
    assert(allclose(sol[0], q))
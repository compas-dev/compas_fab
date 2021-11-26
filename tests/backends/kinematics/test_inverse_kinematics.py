from compas.geometry import Frame
from compas.robots.configuration import Configuration
from compas_fab.robots.ur5 import Robot
from compas_fab.backends.kinematics import AnalyticalInverseKinematics


def test_inverse_kinematics():
    ik = AnalyticalInverseKinematics()
    robot = Robot()
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    solutions = list(ik.inverse_kinematics(robot, frame_WCF, start_configuration=None, group=None, options=None))
    assert(len(solutions) == 8)
    joint_positions, _ = solutions[0]
    correct = Configuration.from_revolute_values((0.022, 4.827, 1.508, 1.126, 1.876, 3.163))
    assert(correct.close_to(Configuration.from_revolute_values(joint_positions)))






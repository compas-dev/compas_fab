import compas
from compas.geometry import Frame
from compas.robots.configuration import Configuration
import compas_fab
from compas_fab.robots.ur5 import Robot
from compas_fab.robots import RobotSemantics
from compas_fab.backends.kinematics import AnalyticalInverseKinematics

if not compas.IPY:
    from compas_fab.backends.kinematics.client import AnalyticalPyBulletClient

urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

def test_inverse_kinematics():
    ik = AnalyticalInverseKinematics()
    robot = Robot()
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    solutions = list(ik.inverse_kinematics(robot, frame_WCF, start_configuration=None, group=None, options=None))
    assert(len(solutions) == 8)
    joint_positions, _ = solutions[0]
    correct = Configuration.from_revolute_values((0.022, 4.827, 1.508, 1.126, 1.876, 3.163))
    assert(correct.close_to(Configuration.from_revolute_values(joint_positions)))


def test_kinematics_client():
    if compas.IPY:
        return
    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))
    with AnalyticalPyBulletClient(connection_type='direct') as client:
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)
        solutions = list(robot.iter_inverse_kinematics(frame_WCF, options={"check_collision": True, "keep_order": False}))
        assert(len(solutions) == 5)


def test_kinematics_cartesian():
    if compas.IPY:
        return
    frames_WCF = [Frame((0.407, 0.073, 0.320), (0.922, 0.000, 0.388), (0.113, 0.956, -0.269)),
                  Frame((0.404, 0.057, 0.324), (0.919, 0.000, 0.394), (0.090, 0.974, -0.210)),
                  Frame((0.390, 0.064, 0.315), (0.891, 0.000, 0.454), (0.116, 0.967, -0.228)),
                  Frame((0.388, 0.079, 0.309), (0.881, 0.000, 0.473), (0.149, 0.949, -0.278)),
                  Frame((0.376, 0.087, 0.299), (0.850, 0.000, 0.528), (0.184, 0.937, -0.296))]

    with AnalyticalPyBulletClient(connection_type='direct') as client:
        robot = client.load_robot(urdf_filename)
        client.load_semantics(robot, srdf_filename)
        start_configuration = list(robot.iter_inverse_kinematics(frames_WCF[0], options={"check_collision": True, "keep_order": False}))[-1]
        trajectory = robot.plan_cartesian_motion(frames_WCF, start_configuration=start_configuration)
        assert(trajectory.fraction == 1.)

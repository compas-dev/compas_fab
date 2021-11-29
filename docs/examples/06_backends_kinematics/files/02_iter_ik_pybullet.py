from compas.geometry import Frame
import compas_fab
from compas_fab.robots import RobotSemantics
from compas_fab.backends.kinematics import AnalyticalInverseKinematics
from compas_fab.backends import PyBulletClient


urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))


with PyBulletClient(connection_type='direct') as client:

    # Load UR5
    robot = client.load_robot(urdf_filename)
    robot.semantics = RobotSemantics.from_srdf_file(srdf_filename, robot.model)
    # Update disabled collisions
    client.disabled_collisions = robot.semantics.disabled_collisions

    ik = AnalyticalInverseKinematics(client)
    # set a new IK function
    client.inverse_kinematics = ik.inverse_kinematics

    for solution in robot.iter_inverse_kinematics(frame_WCF, options={"check_collision": True, "keep_order": True}):
        print(solution)

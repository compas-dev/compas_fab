import compas_fab
from compas.geometry import Frame
from compas_fab.backends.kinematics.client import AnalyticalPyBulletClient

urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

with AnalyticalPyBulletClient(connection_type='direct') as client:
    robot = client.load_robot(urdf_filename)
    client.load_semantics(robot, srdf_filename)

    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))

    options = {"solver": "ur5", "check_collision": True, "keep_order": True}
    # 8 solutions, `None` are those in collision
    for config in robot.iter_inverse_kinematics(frame_WCF, options=options):
        print(config)

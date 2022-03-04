from compas.geometry import Frame
from compas_fab.backends import AnalyticalPyBulletClient


with AnalyticalPyBulletClient(connection_type='direct') as client:
    robot = client.load_ur5(load_geometry=True)

    frame_WCF = Frame((0.381, 0.093, 0.382), (0.371, -0.292, -0.882), (0.113, 0.956, -0.269))

    options = {"solver": "ur5", "check_collision": True, "keep_order": True}
    # 8 solutions, `None` are those in collision
    for config in robot.iter_inverse_kinematics(frame_WCF, options=options):
        print(config)

import compas_fab
from compas.geometry import Frame
from compas_fab.backends import PyBulletClient
from compas_fab.backends import InverseKinematicsError

with PyBulletClient(connection_type='direct') as client:
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    robot = client.load_robot(urdf_filename)

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    start_configuration = robot.zero_configuration()

    options = dict(max_results=20, high_accuracy_threshold=1e-6, high_accuracy_max_iter=20)
    for config in robot.iter_inverse_kinematics(frame_WCF, start_configuration, options=options):
        print("Found configuration", config)

import matplotlib.pyplot as plt
from compas.geometry import Frame
import compas_fab
from compas_fab.backends.kinematics.client import AnalyticalPyBulletClient

urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
srdf_filename = compas_fab.get('universal_robot/ur5_moveit_config/config/ur5.srdf')

frames_WCF = [Frame((0.407, 0.073, 0.320), (0.922, 0.000, 0.388), (0.113, 0.956, -0.269)),
              Frame((0.404, 0.057, 0.324), (0.919, 0.000, 0.394), (0.090, 0.974, -0.210)),
              Frame((0.390, 0.064, 0.315), (0.891, 0.000, 0.454), (0.116, 0.967, -0.228)),
              Frame((0.388, 0.079, 0.309), (0.881, 0.000, 0.473), (0.149, 0.949, -0.278)),
              Frame((0.376, 0.087, 0.299), (0.850, 0.000, 0.528), (0.184, 0.937, -0.296))]

with AnalyticalPyBulletClient(connection_type='direct') as client:
    robot = client.load_robot(urdf_filename)
    client.load_semantics(robot, srdf_filename)

    options = {"solver": "ur5", "check_collision": True}
    start_configuration = list(robot.iter_inverse_kinematics(frames_WCF[0], options=options))[-1]
    trajectory = robot.plan_cartesian_motion(frames_WCF, start_configuration=start_configuration, options=options)
    assert(trajectory.fraction == 1.)

    j = [c.joint_values for c in trajectory.points]
    plt.plot(j)
    plt.show()

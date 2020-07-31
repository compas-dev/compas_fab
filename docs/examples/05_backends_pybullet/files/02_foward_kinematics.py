import compas_fab

from compas_fab.backends import PyBulletClient
from compas_fab.robots import Configuration

with PyBulletClient() as client:
    urdf_filename = compas_fab.get('universal_robot/ur_description/urdf/ur5.urdf')
    robot = client.load_robot(urdf_filename)

    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.])
    options = {'robot': robot}

    frame_WCF = client.forward_kinematics(configuration, options=options)

    print("Frame in the world coordinate system")
    print(frame_WCF)

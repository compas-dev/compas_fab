import compas_fab
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient

with PyBulletClient() as client:
    urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
    robot = client.load_robot(urdf_filename)

    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.0])

    frame_WCF = robot.forward_kinematics(configuration)

    print("Frame in the world coordinate system")
    print(frame_WCF)

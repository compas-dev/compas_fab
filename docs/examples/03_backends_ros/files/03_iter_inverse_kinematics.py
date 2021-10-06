from compas.geometry import Frame
from compas_fab.backends import RosClient

with RosClient() as client:
    robot = client.load_robot()
    assert robot.name == 'ur5'

    frame_WCF = Frame([0.3, 0.1, 0.5], [1, 0, 0], [0, 1, 0])
    start_configuration = robot.zero_configuration()

    for config in robot.iter_inverse_kinematics(frame_WCF, start_configuration, options=dict(max_results=100)):
        print("Found configuration", config)

from compas_fab.backends import RosClient
from compas.scene import Scene

with RosClient() as ros:
    # Load complete model from ROS
    robot = ros.load_robot(load_geometry=True, precision=12)

    # Visualize robot
    scene = Scene()
    scene_object = scene.add(robot.model)
    scene.draw()

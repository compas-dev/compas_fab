from compas.scene import Scene
from compas_fab.backends import RosClient

# NOTE: Requires a running ROS backend.  See the ROS Examples section for setup.
with RosClient() as ros:
    # Load complete robot model from ROS (geometry is in meters).
    robot = ros.load_robot(load_geometry=True)

    # Create a scene and obtain a scene object for the robot model.
    scene = Scene()
    scene_object = scene.add(robot.model)

    # Assign the scene object so the robot knows about the visualization.
    robot.scene_object = scene_object

    # Scale from meters to millimeters.
    # Calling scale() *after* setting the scene object ensures that both
    # the internal model geometry and the visualization are updated together.
    robot.scale(1000)

    scene.draw()

# This example requires compas_viewer to be installed. You can install it with pip:
# `pip install compas_viewer`

# This example shows how to visualize a robot using compas_viewer and how to update the robot's configuration every second.
# The configuration is randomly generated and the robot's flange center is marked with a permanent red point.

from compas_fab.robots import RobotLibrary
from compas_viewer import Viewer
from compas_robots.viewer.scene.robotmodelobject import RobotModelObject
from compas.colors import Color

viewer = Viewer()

# Load robot from RobotLibrary (Uncomment the robot you want to visualize)
# robot = RobotLibrary.ur5(load_geometry=True)
robot = RobotLibrary.ur10e(load_geometry=True)
# robot = RobotLibrary.abb_irb4600_40_255(load_geometry=True)

# The RobotModelObject for compas_viewer is automatically created when adding the model to the scene
object = viewer.scene.add(robot.model)  # type: RobotModelObject


# Update the robot's configuration at 10ms intervals
@viewer.on(interval=10)
def random_config(frame):
    random_config = robot.random_configuration()
    print(f"Frame {frame} - Random configuration: {random_config}")
    object.update_joints(random_config)

    # Add a point to mark the flange center of the robot
    fk_result = robot.forward_kinematics(random_config)
    point_object = viewer.scene.add(fk_result.point, pointsize=2.0, pointcolor=Color.red())
    point_object.init()

    # Update the viewer
    viewer.renderer.update()


# Start the viewer, and block the code until it is closed.
viewer.show()

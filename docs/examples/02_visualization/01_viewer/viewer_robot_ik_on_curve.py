# This example requires compas_viewer to be installed. You can install it with pip:
# `pip install compas_viewer`

# This example will use the PyBulletPlanner to perform inverse_kinematics on a point that moves along a curve.
# The IK calculation is performed within the viewer's update event.
# The planned configuration is used as the `start_configuration` for the next iteration to avoid sudden jumps in the robot's movement.

from compas.colors import Color
from compas.geometry import Bezier
from compas.geometry import Point
from compas.geometry import Polyline
from compas.geometry import Frame

from compas_fab.robots import RobotLibrary
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner

from compas_robots.viewer.scene.robotmodelobject import RobotModelObject

from compas_viewer import Viewer
from compas_viewer.layout import Slider
from compas_viewer.scene import PointObject

viewer = Viewer()

# Load robot from RobotLibrary (Uncomment the robot you want to visualize)
# robot = RobotLibrary.ur5(load_geometry=True)
# robot = RobotLibrary.ur10e(load_geometry=True)
robot = RobotLibrary.abb_irb4600_40_255(load_geometry=True)

# The curve to place a point on
curve = Bezier([[-0.9, 1.0, 0.5], [0.1, 1.6, 1.2], [1.3, -1, 0.4], [1.7, 0, 1.1]])

# Point and Curve Object
pointobj: PointObject = viewer.scene.add(Point(*curve.point_at(0)), pointsize=20, pointcolor=Color.red(), show_points=True)  # type: ignore
curveobj = viewer.scene.add(Polyline(curve.to_polyline()), linewidth=2, linecolor=Color.blue(), show_points=False)

with PyBulletClient(connection_type="direct") as client:
    robot = client.load_existing_robot(robot)
    planner = PyBulletPlanner(client)
    planning_group = robot.main_group_name

    # The RobotModelObject for compas_viewer is automatically created when adding the model to the scene
    robot_object = viewer.scene.add(robot.model)  # type: RobotModelObject
    start_configuration = robot.zero_configuration()

    # Update the robot's configuration at each frame. Setting it at 1ms interval effectively means as fast as possible.
    @viewer.on(interval=1)
    def update(frame):
        STEPS = 100
        # eval_value goes from 0.0 to 1.0 and back to 0.0, repeating every 100 frames. W waveform
        eval_value = (frame % STEPS) / (STEPS / 2)
        if eval_value > 1:
            eval_value = 2 - eval_value
        pointobj.geometry = curve.point_at(eval_value)
        pointobj.init()
        pointobj.update()
        viewer.renderer.update()

        # Create a target frame from the point on the curve
        target_frame = Frame(pointobj.geometry, [1, 0, 0], [0, -1, 0])

        # Add a point to mark the flange center of the robot
        global start_configuration
        configuration = robot.inverse_kinematics(target_frame, start_configuration)
        robot_object.update_joints(configuration)

        # using the planned configuration as the start for the next iteration
        start_configuration = configuration

        # Update the viewer
        viewer.renderer.update()

    # Start the viewer, and block the code until it is closed.
    viewer.show()

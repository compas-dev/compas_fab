# This example does not work with IronPython
from compas_fab.robots import RobotCellLibrary
from compas_robots.viewer.scene.robotmodelobject import RobotModelObject
from compas_robots.model import Joint
from compas_viewer.components import Slider
from compas_viewer import Viewer

# Initialize the viewer
viewer = Viewer()
viewer.renderer.rendermode = "lighted"
viewer.ui.sidedock.show = True

# Load robot from RobotCellLibrary
# RobotCellLibrary also contains .ur5(), .ur10e(), abb_irb120_3_58(), abb_irb4600_40_255(), .rfl(), .panda()
robot_cell, robot_cell_state = RobotCellLibrary.ur5()
model = robot_cell.robot_model

start_configuration = model.zero_configuration()
robot_object: RobotModelObject = viewer.scene.add(model, show_lines=False, configuration=start_configuration)  # type: ignore


# Callback function when the slider is moved to update the robot's joint values
def make_rotate_function(joint_name):
    def rotate(slider: Slider, value: float):
        config = robot_object.configuration
        config[joint_name] = value
        robot_object.update_joints(config)

    return rotate


# Create one slider for each joint
for joint in robot_cell.get_configurable_joints():
    starting_val = start_configuration[joint.name]
    print(joint.name, Joint.SUPPORTED_TYPES[joint.type], joint.limit.lower, joint.limit.upper, starting_val)
    # Units are in radians or meters
    step_size = (joint.limit.upper - joint.limit.lower) / 100
    rotate_function = make_rotate_function(joint.name)
    viewer.ui.sidedock.add(
        Slider(
            title=joint.name + " (" + Joint.SUPPORTED_TYPES[joint.type] + ")",
            starting_val=starting_val,
            min_val=joint.limit.lower,
            max_val=joint.limit.upper,
            step=step_size,
            action=rotate_function,
        )
    )

# configuration = model.zero_configuration()
robot_object.update_joints(start_configuration)

viewer.show()

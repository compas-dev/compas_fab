from compas_robots import Configuration
from compas_robots import RobotModel

import compas_fab

# Load robot from URDF file, geometry is not required for forward kinematics
urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
robot_model = RobotModel.from_urdf_file(urdf_filename)  # type: RobotModel

# Create a configuration with joint values
configuration = robot_model.zero_configuration()
configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]
frame = robot_model.forward_kinematics(configuration, "tool0")
# The frame result is correct
print("Configuration: {}".format(configuration))
print("Correct Frame of tool0: {}".format(frame))
# This is the expected output:
assert str(frame.point) == "Point(x=0.300, y=0.100, z=0.500)"

# Do not use a Configuration object without joint names
configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.0])
frame = robot_model.forward_kinematics(configuration, "tool0")
# It will produce incorrect results
print("Configuration: {}".format(configuration))
print("Incorrect Frame of tool0: {}".format(frame))

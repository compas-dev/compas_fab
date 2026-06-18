from compas_fab.robots import RobotLibrary

# Load a UR5 robot with geometry.
# Robot models loaded from URDF/ROS are defined in meters by convention.
robot = RobotLibrary.ur5(load_geometry=True)

print("Default scale factor:", robot.scale_factor)

# Scale the robot geometry from meters to millimeters.
# Pass the desired scale factor as an absolute multiplier.
robot.scale(1000)

print("Scale factor after scaling to mm:", robot.scale_factor)

# scale() applies an *absolute* (non-cumulative) factor.
# Calling scale(1000) a second time leaves the robot at 1000x, not 1000*1000x.
robot.scale(1000)
print("Scale factor (repeated call, still 1000):", robot.scale_factor)

# Use a different value to change the scale at any point.
robot.scale(10)
print("Scale factor after changing to 10:", robot.scale_factor)

# Revert to the original meters-based scale.
robot.scale(1)
print("Scale factor after reverting to meters:", robot.scale_factor)

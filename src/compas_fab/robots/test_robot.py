# Import necessary modules
try:
    from compas_fab.robots.ur5 import Robot  # Attempt to import the UR5 robot directly
    robot = Robot()  # Instantiate the UR5 robot
except ImportError:  # Catch the specific ImportError
    from compas_fab.robots import RobotLibrary  # Fall back to RobotLibrary
    robot = RobotLibrary.ur5()  # Instantiate the UR5 robot from the library

# Check if robot has been initialized successfully
if robot:
    # List all group names in the robot model
    print("Available Groups:", robot.group_names)  # Print available group names
else:
    print("Failed to initialize the robot.")

if __name__ == "__main__":
    # Place the script's main code here if needed
    pass  # Placeholder for any additional code

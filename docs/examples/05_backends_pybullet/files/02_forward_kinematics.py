import compas_fab
from compas_robots import Configuration
from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState
from compas_fab.robots import RobotLibrary


# #############################################
# Headless (no-GUI) forwards kinematics example
# #############################################

# 'direct' mode
with PyBulletClient("direct") as client:
    # The robot in this example is loaded from a URDF file
    robot = RobotLibrary.ur5()

    # The planner object is needed to call the forward kinematics function
    planner = PyBulletPlanner(client)

    # ----------------
    # FK without tools
    # ----------------

    # This is a simple robot cell with only the robot
    robot_cell = RobotCell(robot)
    planner.set_robot_cell(robot_cell)

    configuration = Configuration.from_revolute_values([-2.238, -1.153, -2.174, 0.185, 0.667, 0.0])
    # The `RobotCellState.from_robot_configuration` method can be used when the robot is the only element in the cell
    robot_cell_state = RobotCellState.from_robot_configuration(robot, configuration)
    # In this demo, the default planning group is used for the forward kinematics
    frame_WCF = planner.forward_kinematics(robot_cell_state)

    print("Robot flange frame of the default planning group in the world coordinate system:")
    print(frame_WCF)
    print(" ")

    # ---------------------------------
    # FK for all the links in the robot
    # ---------------------------------

    for link_name in robot.get_link_names():
        frame_WCF = planner.forward_kinematics(robot_cell_state, options={"link": link_name})
        print(f"Frame of link '{link_name}' : {frame_WCF}")

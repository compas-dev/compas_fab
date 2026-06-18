from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode

# Starting the PyBulletClient with the "direct" mode means that the GUI is not shown
with PyBulletClient("direct") as client:

    # The robot in this example is loaded from a URDF file
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()

    # The planner object is needed to call the forward kinematics function
    planner = PyBulletPlanner(client)

    # ----------------
    # FK without tools
    # ----------------
    # This is a simple robot cell with only the robot
    planner.set_robot_cell(robot_cell)

    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    # In this demo, the default planning group is used for the forward kinematics
    frame_WCF = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)

    print("Robot flange frame of the default planning group in the world coordinate system:")
    print(frame_WCF)
    print(" ")

    # ---------------------------------
    # FK for all the links in the robot
    # ---------------------------------
    for link_name in robot_cell.get_link_names():
        frame_WCF = planner.forward_kinematics_to_link(robot_cell_state, link_name)
        print("Frame of link '{}' : {}".format(link_name, frame_WCF))

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCellLibrary

# Starting the PyBulletClient with the "direct" mode means that the GUI is not shown
with PyBulletClient("direct") as client:

    # The robot in this example is loaded from a URDF file
    # This is a simple robot cell with only the robot
    robot_cell, robot_cell_state = RobotCellLibrary.ur5()

    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # Change the robot configuration to a specific one
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    # FK for all the links in the robot
    for link_name in robot_cell.get_link_names():
        frame_WCF = planner.forward_kinematics_to_link(robot_cell_state, link_name)
        print("Frame of link '{}' : {}".format(link_name, frame_WCF))

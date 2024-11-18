from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    planner = MoveItPlanner(client)

    robot_cell_state = robot_cell.default_cell_state()
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    for link_name in robot_cell.get_link_names():
        frame_lcf_wcf = planner.forward_kinematics_to_link(robot_cell_state, link_name)
        print("Link ({}) LCF relative to the WCF: {}".format(link_name, frame_lcf_wcf))

from compas_fab.backends import RosClient

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    robot_cell.print_info()

    assert robot_cell.robot_model.name == "ur5_robot"

from compas_fab.backends import RosClient

with RosClient() as ros:
    robot_cell = ros.load_robot_cell(load_geometry=True, precision=12)

    print(robot_cell.robot_model)

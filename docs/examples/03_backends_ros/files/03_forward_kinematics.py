from compas_fab.robots import TargetMode
from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    assert robot_cell.robot_model.name == "ur5_robot"
    planner = MoveItPlanner(client)

    robot_cell_state = robot_cell.default_cell_state()
    robot_cell_state.robot_configuration.joint_values = [-2.238, -1.153, -2.174, 0.185, 0.667, 0.0]

    # When using the forward_kinematics() method with TargetMode.ROBOT, the last link of the robot's main group is used.
    frame_WCF = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)

    print("Frame in the world coordinate system")
    print(frame_WCF)

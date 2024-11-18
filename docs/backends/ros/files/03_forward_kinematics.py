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
    frame_pcf_wcf = planner.forward_kinematics(robot_cell_state, TargetMode.ROBOT)

    print("Last link name: {}".format(robot_cell.get_end_effector_link_name()))
    print("PCF Frame relative to the WCF: {}".format(frame_pcf_wcf))

"""
Output:

>>> Last link name: tool0
>>> PCF Frame relative to the WCF: Frame(point=Point(x=0.300, y=0.100, z=0.500), xaxis=Vector(x=-0.000, y=-1.000, z=0.000), yaxis=Vector(x=-0.000, y=-0.000, z=-1.000))
"""

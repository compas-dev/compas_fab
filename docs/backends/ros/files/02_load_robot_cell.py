from compas_fab.backends import RosClient
from compas_fab.backends import MoveItPlanner

with RosClient() as client:
    robot_cell = client.load_robot_cell()
    planner = MoveItPlanner(client)
    robot_cell.print_info()

"""
Output:
>>> The robot's name is 'ur5_robot'.
>>> The robot's joints are:
>>>         * 'base_link-base_link_inertia' is of type 'fixed' (Fixed)
>>>         * 'base_link-base_fixed_joint' is of type 'fixed' (Fixed)
>>>         * 'shoulder_pan_joint' is of type 'revolute' and has limits [6.283, -6.283] (Configurable)
>>>         * 'shoulder_lift_joint' is of type 'revolute' and has limits [6.283, -6.283] (Configurable)
>>>         * 'elbow_joint' is of type 'revolute' and has limits [3.142, -3.142] (Configurable)
>>>         * 'wrist_1_joint' is of type 'revolute' and has limits [6.283, -6.283] (Configurable)
>>>         * 'wrist_2_joint' is of type 'revolute' and has limits [6.283, -6.283] (Configurable)
>>>         * 'wrist_3_joint' is of type 'revolute' and has limits [6.283, -6.283] (Configurable)
>>>         * 'wrist_3-flange' is of type 'fixed' (Fixed)
>>>         * 'flange-tool0' is of type 'fixed' (Fixed)
>>> The robot's links are:
>>> ['base_link', 'base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'base', 'flange', 'tool0']
>>> The planning groups are: ['manipulator', 'endeffector']
>>> The main planning group is 'manipulator'.
>>>        - base link's name is 'base_link'
>>>        - end-effector's link name is 'tool0'.>>> The following tools are present:
>>> The following tools are present:
>>> []
>>> The following rigid bodies are present:
>>> []
"""

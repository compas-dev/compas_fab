from compas_fab.robots import RobotCellLibrary

# Load from the robot cell library
robot_cell, _ = RobotCellLibrary.ur10e(load_geometry=False)

# Print all available planning groups
for group_name, group in robot_cell.robot_semantics.groups.items():
    print("Planning group: '{}'".format(group_name))
    print("  Joints: {}".format(group["joints"]))
    print("  Links: {}".format(group["links"]))

"""
Output:
>>> Planning group: 'manipulator'
>>>   Joints: ['base_link-base_link_inertia', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'wrist_3-flange', 'flange-tool0']
>>>   Links: ['base_link', 'base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'flange', 'tool0']
"""

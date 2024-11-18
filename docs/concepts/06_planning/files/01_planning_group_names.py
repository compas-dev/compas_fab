from compas_fab.robots import RobotCellLibrary

# Load from the robot cell library
robot_cell, _ = RobotCellLibrary.rfl(load_geometry=False)

# Print all available planning groups
group_names = robot_cell.robot_semantics.group_names
for group_name in group_names:
    base_link = robot_cell.get_base_link_name(group_name)
    end_effector_link = robot_cell.get_end_effector_link_name(group_name)
    print("Planning group: '{}'".format(group_name))
    print("    base_link: '{}' end_effector_link: '{}'".format(base_link, end_effector_link))

"""
Output:
>>> Planning group: 'robot11'
>>>     base_link: 'robot11_base_link' end_effector_link: 'robot11_tool0'
>>> Planning group: 'robot12'
>>>     base_link: 'robot12_base_link' end_effector_link: 'robot12_tool0'
>>> Planning group: 'robot11_eaXYZ'
>>>     base_link: 'x_rail' end_effector_link: 'robot11_tool0'
>>> Planning group: 'robot12_eaYZ'
>>>     base_link: 'bridge1' end_effector_link: 'robot12_tool0'
>>> Planning group: 'robot21'
>>>     base_link: 'robot21_base_link' end_effector_link: 'robot21_tool0'
>>> Planning group: 'robot22'
>>>     base_link: 'robot22_base_link' end_effector_link: 'robot22_tool0'
>>> Planning group: 'robot21_eaXYZ'
>>>     base_link: 'x_rail' end_effector_link: 'robot21_tool0'
>>> Planning group: 'robot22_eaYZ'
>>>     base_link: 'bridge2' end_effector_link: 'robot22_tool0'
>>> Planning group: 'robot11_eaYZ'
>>>     base_link: 'bridge1' end_effector_link: 'robot11_tool0'
>>> Planning group: 'robot21_eaYZ'
>>>     base_link: 'bridge2' end_effector_link: 'robot21_tool0'
>>> Planning group: 'robot12_eaXYZ'
>>>     base_link: 'x_rail' end_effector_link: 'robot12_tool0'
>>> Planning group: 'robot22_eaXYZ'
>>>     base_link: 'x_rail' end_effector_link: 'robot22_tool0'
"""

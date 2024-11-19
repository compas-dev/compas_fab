# This example file demonstrates how to print the chain of links and joints in a robot model.

from compas_fab.robots import RobotCellLibrary
from compas_robots.model import Joint

# RobotCellLibrary also contains .ur5(), .ur10e(), abb_irb120_3_58(), abb_irb4600_40_255(), .rfl(), .panda()
robot_cell, robot_cell_state = RobotCellLibrary.panda(load_geometry=False)

model = robot_cell.robot_model
print("Robot Model Chain:")

# ----------------------------------------------
# Example 1: Print the chain of links and joints
# ----------------------------------------------

base_link_name = model.get_base_link_name()
print("Base Link: {}".format(base_link_name))


# Iteratively print the chain of links and joints
def print_joint(joint, level=1):
    # type: (Joint, int) -> None
    link = joint.child_link
    print("-" * level + "Joint: {} Link: {}".format(joint.name, link.name))
    for child_joint in link.joints:
        print_joint(child_joint, level + 1)


first_joint = model.joints[0]
print_joint(first_joint)

# ---------------------------------------------------------------------------------
# Example 2: Print the chain and highlight the links and joints in a planning group
# ---------------------------------------------------------------------------------

print("")


def print_planning_group_chain(group):
    print("Planning Group: {}".format(group))
    base_link = robot_cell.get_base_link_name(group)
    print("Base Link: {}".format(base_link))
    tip_link = robot_cell.get_end_effector_link_name(group)
    print("Tip Link: {}".format(tip_link))
    print("--------------------")
    group_object = robot_cell.robot_semantics.groups[group]
    joints_in_group = group_object["joints"]
    links_in_group = group_object["links"]

    # Iteratively print the chain of links and joints
    def print_joint(joint, level=1):
        # type: (Joint, int) -> None
        link = joint.child_link
        line = "-" * level

        # Joint info
        if joint.name in joints_in_group:
            line += " [Joint: {}]".format(joint.name)
        else:
            line += " Joint: {}".format(joint.name)

        # Type of joint
        line += " ("
        if joint.is_configurable():
            line += "Configurable"
        elif joint.mimic:
            line += "Mimic"
        else:
            line += "Fixed"
        line += " {})".format(Joint.SUPPORTED_TYPES[joint.type])

        # Link info
        if link.name in links_in_group:
            line += " [Link: {}]".format(link.name)
        else:
            line += " Link: {}".format(link.name)
        print(line)
        for child_joint in link.joints:
            print_joint(child_joint, level + 1)

    print("Base Link: {}".format(model.get_base_link_name()))
    first_joint = model.joints[0]
    print_joint(first_joint)
    print("")


for group in robot_cell.robot_semantics.groups:
    print_planning_group_chain(group)

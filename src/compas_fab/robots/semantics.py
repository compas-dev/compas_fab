from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.files import XML

__all__ = [
    'RobotSemantics',
]


class RobotSemantics(object):
    """Represents semantic information of a robot.

    The semantic model is based on the
    `Semantic Robot Description Format` (`SRDF`_).

    References
    ----------

    .. _SRDF: https://wiki.ros.org/srdf

    """

    def __init__(self, root, urdf_robot):
        self.root = root
        self.group_names = self.__get_group_names()
        self.passive_joints = self.__get_passive_joints()
        self.end_effectors = self.__get_end_effectors()
        self._group_dict = {}
        self.main_group = None
        self.urdf_robot = urdf_robot
        self.__source_attributes()

    @classmethod
    def from_srdf_file(cls, file, urdf_robot):
        xml = XML.from_file(file)
        return cls(xml.root, urdf_robot)

    @classmethod
    def from_srdf_string(cls, text, urdf_robot):
        xml = XML.from_string(text)
        return cls(xml.root, urdf_robot)

    def __get_group_link_names(self, group):
        link_names = []
        for link in group.iter('link'):
            name = link.attrib['name']
            if name not in link_names:
                link_names.append(name)
        for chain in group.iter('chain'):
            for link in self.urdf_robot.iter_link_chain(chain.attrib['base_link'], chain.attrib['tip_link']):
                name = link.name
                if name not in link_names:
                    link_names.append(name)
        for joint in group.iter('joint'):
            joint = self.urdf_robot.get_joint_by_name(joint.attrib['name'])
            if joint:
                name = joint.parent.link
                if name not in link_names:
                    link_names.append(name)
        for subgroup in group.iter('group'):
            if subgroup.attrib['name'] != group.attrib['name']:
                subgroup_link_names = self.__get_group_link_names(subgroup)
                for name in subgroup_link_names:
                    if name not in link_names:
                        link_names.append(name)
        return link_names

    def __get_group_joints(self, group):
        link_names = self.__get_group_link_names(group)
        joints = []
        for name in link_names:
            link = self.urdf_robot.get_link_by_name(name)
            for joint in link.joints:
                if joint.is_configurable() and joint.name not in self.passive_joints:
                    joints.append(joint)
        return joints

    def __get_group_joint_names(self, group):
        joints = self.__get_group_joints(group)
        return [j.name for j in joints]

    def __source_attributes(self):

        gnames = []
        glenth = []

        group_dict = {}
        for group in self.root.iter('group'):
            name = group.attrib['name']
            group_dict[name] = {}
            link_names = self.__get_group_link_names(group)
            joint_names = self.__get_group_joint_names(group)
            group_dict[name]["links"] = link_names
            group_dict[name]["joints"] = joint_names
            gnames.append(name)
            glenth.append(len(link_names))

        idx = glenth.index(max(glenth))

        self._group_dict = group_dict
        self.main_group_name = gnames[idx]

    def __get_group_names(self):
        return [group.attrib['name'] for group in self.root.iter('group')]

    def __get_passive_joints(self):
        return [pjoint.attrib['name'] for pjoint in self.root.iter('passive_joint')]

    def __get_end_effectors(self):
        return [ee.attrib['parent_link'] for ee in self.root.iter('end_effector')]

    def get_end_effector_link_name(self, group=None):
        if not group:
            group = self.main_group_name
        return self._group_dict[group]["links"][-1]

    def get_base_link_name(self, group=None):
        if not group:
            group = self.main_group_name
        return self._group_dict[group]["links"][0]

    def get_configurable_joints(self, group=None):
        joint_names = self.get_configurable_joint_names(group)
        return [self.urdf_robot.get_joint_by_name(name) for name in joint_names]

    def get_configurable_joint_names(self, group=None):
        if not group:
            group = self.main_group_name
        return self._group_dict[group]["joints"]


if __name__ == "__main__":

    import os
    from compas.robots import Robot as UrdfRobot

    path = r"C:\Users\rustr\workspace\robot_description"

    for item in os.listdir(path):
        fullpath = os.path.join(path, item)
        if os.path.isdir(fullpath) and item[0] != ".":
            urdf_file = os.path.join(fullpath, 'urdf', 'robot_description.urdf')
            srdf_file = os.path.join(fullpath, 'robot_description_semantic.srdf')

            urdf_model = UrdfRobot.from_urdf_file(urdf_file)
            srdf_model = RobotSemantics.from_srdf_file(srdf_file, urdf_model)

            print(urdf_model.name)
            print("group_names:", srdf_model.group_names)
            print("main_group_name:", srdf_model.main_group_name)
            print("base_link_name:", srdf_model.get_base_link_name())
            print("ee_link_name:", srdf_model.get_end_effector_link_name())
            print("configurable_joints:", srdf_model.get_configurable_joint_names())
            print()

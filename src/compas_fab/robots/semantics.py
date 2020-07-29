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
        self._group_dict = {}
        self.main_group = None
        self.urdf_robot = urdf_robot
        self.__source_attributes()

    @property
    def group_names(self):
        return [group.attrib['name'] for group in self.root.findall('group')]

    @property
    def passive_joints(self):
        return [pjoint.attrib['name'] for pjoint in self.root.iter('passive_joint')]

    @property
    def end_effectors(self):
        return [ee.attrib['parent_link'] for ee in self.root.findall('end_effector')]

    @property
    def disabled_collisions(self):
        return {
            frozenset([dc.attrib['link1'], dc.attrib['link2']])
            for dc in self.root.iter('disable_collisions')
        }

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
        for link in group.findall('link'):
            name = link.attrib['name']
            if name not in link_names:
                link_names.append(name)
        for chain in group.findall('chain'):
            for link in self.urdf_robot.iter_link_chain(chain.attrib['base_link'], chain.attrib['tip_link']):
                if link.name not in link_names:
                    link_names.append(link.name)
        for joint in group.findall('joint'):
            joint = self.urdf_robot.get_joint_by_name(joint.attrib['name'])
            if joint:
                name = joint.parent.link
                if name not in link_names:
                    link_names.append(name)
        for subgroup in group.findall('group'):
            if subgroup.attrib['name'] != group.attrib['name']:
                # find group element at top level
                for top_group_elem in self.root.findall('group'):
                    if top_group_elem.attrib['name'] == subgroup.attrib['name']:
                        subgroup_link_names = self.__get_group_link_names(top_group_elem)
                        for name in subgroup_link_names:
                            if name not in link_names:
                                link_names.append(name)
        return link_names

    def __get_group_elem_by_name(self, group_name):
        for group_elem in self.root.findall('group'):
            if group_elem.attrib['name'] == group_name:
                return group_elem

    def __get_group_joint_names(self, group):
        joint_names = []
        for link in group.findall('link'):
            link = self.urdf_robot.get_link_by_name(link.attrib['name'])
            for joint in link.joints:
                if joint.name not in joint_names:
                    joint_names.append(joint.name)
        for chain in group.findall('chain'):
            for joint in self.urdf_robot.iter_joint_chain(chain.attrib['base_link'], chain.attrib['tip_link']):
                if joint.name not in joint_names:
                    joint_names.append(joint.name)
        for joint in group.findall('joint'):
            if joint.attrib['name'] not in joint_names:
                joint_names.append(joint.attrib['name'])
        for subgroup in group.findall('group'):
            if subgroup.attrib['name'] != group.attrib['name']:
                # find group element at top level
                top_group_elem = self.__get_group_elem_by_name(subgroup.attrib['name'])
                subgroup_joint_names = self.__get_group_joint_names(top_group_elem)
                for name in subgroup_joint_names:
                    if name not in joint_names:
                        joint_names.append(name)
        return joint_names

    def __source_attributes(self):

        gnames = []
        glenth = []

        group_dict = {}
        for group in self.root.findall('group'):
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

    def get_end_effector_link_name(self, group=None):
        if not group:
            group = self.main_group_name
        return self._group_dict[group]["links"][-1]

    def get_base_link_name(self, group=None):
        if not group:
            group = self.main_group_name
        return self._group_dict[group]["links"][0]

    def get_all_configurable_joints(self):
        joints = []
        for joint in self.urdf_robot.get_configurable_joints():
            if joint.name not in self.passive_joints:
                joints.append(joint)
        return joints

    def get_configurable_joints(self, group=None):
        if not group:
            group = self.main_group_name
        joints = []
        for name in self._group_dict[group]["joints"]:
            joint = self.urdf_robot.get_joint_by_name(name)
            if joint:
                if joint.is_configurable() and name not in self.passive_joints:
                    joints.append(joint)
        return joints

    def get_configurable_joint_names(self, group=None):
        return [joint.name for joint in self.get_configurable_joints(group)]

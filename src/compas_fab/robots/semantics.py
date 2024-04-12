from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.data import Data
from compas.files import XML

__all__ = [
    "RobotSemantics",
]


class RobotSemantics(Data):
    """Represents semantic information of a robot.

    The semantic model is based on the
    `Semantic Robot Description Format` (`SRDF`_).
    Typically, the RobotSemantics objects are created from an SRDF file (using :meth:`from_srdf_file`)
    or loaded by the backend client (using :meth:`compas_fab.backends.PyBulletClient.load_semantics` or
    :meth:`compas_fab.backends.RosClient.load_robot`).

    Parameters
    ----------
    robot_model : :class:`~compas_fab.robots.RobotModel`
        The robot model.
    groups : :obj:`dict` of (:obj:`str`, :obj:`dict` of (``links`` : :obj:`list` of :obj:`str`, ``joints`` : :obj:`list` of :obj:`str`)), optional
        A nested dictionary defining planning groups.
        The dictionary structure is as follows:

        - Level 1 keys are planning group names : :obj:`str`.
        - Level 2 contains only two keys:

            -   ``links`` is a :obj:`list` of :obj:`str` containing link names
            -   ``joints`` is a :obj:`list` of :obj:`str` containing joint names.

    main_group_name : :obj:`str`, optional
        The name of the main group.
    passive_joints : :obj:`list` of :obj:`str`, optional
        A list of passive joint names.
    end_effectors : :obj:`list` of :obj:`str`, optional
        A list of end effector link names.
    disabled_collisions : :obj:`tuple` of (:obj:`str`, :obj:`str`), optional
        A set of disabled collision pairs.
        The order is not important, i.e. the pair `('link1', 'link2')` is the same as `('link2', 'link1')`.
        Only one pair is needed.
    group_states : dict, optional
        A nested dictionary defining named states for a particular group, in terms of joint values.
        This is useful to define states such as "home" or "folded arms" for the robot.
        The dictionary structure is as follows:

        - Level 1 keys are planning group names : :obj:`str`.
        - Level 2 keys are group state names : :obj:`str`.
        - Level 3 keys are joint names and values are joint values : :obj:`str`.

    Attributes
    ----------
    group_names : :obj:`list` of :obj:`str`, read-only
        Get the names of all planning groups.
    unordered_disabled_collisions : :obj:`set` of :obj:`frozenset`, read-only
        Get the disabled collision pairs as a set of frozensets.

    References
    ----------

    .. _SRDF: https://wiki.ros.org/srdf

    """

    def __init__(
        self,
        robot_model,
        groups=None,
        main_group_name=None,
        passive_joints=None,
        end_effectors=None,
        disabled_collisions=None,
        group_states=None,
    ):
        super(RobotSemantics, self).__init__()
        self.robot_model = robot_model

        self.groups = groups or {}
        self.main_group_name = main_group_name
        self.passive_joints = passive_joints or []
        self.end_effectors = end_effectors or []
        self.disabled_collisions = disabled_collisions or set()
        self.group_states = group_states or {}

    @property
    def __data__(self):
        data = {
            "robot_model": self.robot_model,
            "groups": self.groups,
            "main_group_name": self.main_group_name,
            "passive_joints": self.passive_joints,
            "end_effectors": self.end_effectors,
            "disabled_collisions": sorted(self.disabled_collisions),
            "group_states": self.group_states,
        }
        return data

    @classmethod
    def __from_data__(cls, data):
        robot_model = data.get("robot_model")
        groups = data.get("groups", {})
        main_group_name = data.get("main_group_name")
        passive_joints = data.get("passive_joints", [])
        end_effectors = data.get("end_effectors", [])
        disabled_collisions = data.get("disabled_collisions", set())
        if len(disabled_collisions) > 0:
            disabled_collisions = {tuple(pair) for pair in disabled_collisions}
        group_states = data.get("group_states", {})

        robot_semantics = cls(
            robot_model,
            groups=groups,
            main_group_name=main_group_name,
            passive_joints=passive_joints,
            end_effectors=end_effectors,
            disabled_collisions=disabled_collisions,
            group_states=group_states,
        )
        return robot_semantics

    @property
    def group_names(self):
        return list(self.groups.keys())

    @property
    def unordered_disabled_collisions(self):
        return {frozenset(pair) for pair in self.disabled_collisions}

    @classmethod
    def from_srdf_file(cls, file, robot_model):
        """Create an instance of semantics based on an SRDF file path or file-like object.

        Parameters
        ----------
        file : :obj:`str`
            The path to the SRDF file.
        robot_model : :class:`compas_robots.RobotModel`
            The robot model is needed when loading the semantics.

        Examples
        --------
        >>> from compas_fab.robots import RobotSemantics
        >>> from compas_robots import RobotModel
        >>> urdf_filename = compas_fab.get("robot_library/ur5_robot/urdf/robot_description.urdf")
        >>> srdf_filename = compas_fab.get("robot_library/ur5_robot/robot_description_semantic.srdf")
        >>> robot_model = RobotModel.from_urdf_file(urdf_filename)
        >>> semantics = RobotSemantics.from_srdf_file(srdf_filename, robot_model)
        >>> print(semantics.main_group_name)
        manipulator
        """
        xml = XML.from_file(file)
        return cls.from_xml(xml, robot_model)

    @classmethod
    def from_srdf_string(cls, text, robot_model):
        """Create an instance of semantics based on an SRDF string."""
        xml = XML.from_string(text)
        return cls.from_xml(xml, robot_model)

    @classmethod
    def from_xml(cls, xml, robot_model):
        """Create an instance of semantics based on an XML object."""
        groups = _get_groups(xml.root, robot_model)
        passive_joints = _get_passive_joints(xml.root)
        end_effectors = _get_end_effectors(xml.root)
        disabled_collisions = _get_disabled_collisions(xml.root)
        group_states = _get_group_states(xml.root)

        groups_by_links_len = sorted(groups.items(), key=lambda group: len(group[1]["links"]))
        main_group_name = groups_by_links_len[-1][0]

        return cls(
            robot_model,
            groups=groups,
            main_group_name=main_group_name,
            passive_joints=passive_joints,
            end_effectors=end_effectors,
            disabled_collisions=disabled_collisions,
            group_states=group_states,
        )

    def get_end_effector_link_name(self, group=None):
        """Get the :obj:`str` name of the last link (end effector link) in a planning group.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`str`
            The name of the end effector link.
        """

        if not group:
            group = self.main_group_name
        return self.groups[group]["links"][-1]

    def get_base_link_name(self, group=None):
        """Get the :obj:`str` name of the first link (base link) in a planning group.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`str`
            The name of the base link.
        """
        if not group:
            group = self.main_group_name
        return self.groups[group]["links"][0]

    def get_all_configurable_joints(self):
        """Get all configurable :class:`compas_robots.model.Joint` of the robot.

        Configurable joints are joints that can be controlled,
        aka, not ``Joint.Fixed``, not mimicking another joint and not a passive joint.
        See :meth:`compas_robots.model.Joint.is_configurable` for more details.

        Returns
        -------
        :obj:`list` of :class:`compas_robots.model.Joint`
            A list of configurable joints.

        """

        joints = []
        for joint in self.robot_model.get_configurable_joints():
            if joint.name not in self.passive_joints:
                joints.append(joint)
        return joints

    def get_configurable_joints(self, group=None):
        """Get all configurable :class:`compas_robots.model.Joint` of a planning group.

        Configurable joints are joints that can be controlled,
        aka, not ``Joint.Fixed``, not mimicking another joint and not a passive joint.
        See :meth:`compas_robots.model.Joint.is_configurable` for more details.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :class:`compas_robots.model.Joint`
            A list of configurable joints.

        """
        if not group:
            group = self.main_group_name
        joints = []
        for name in self.groups[group]["joints"]:
            joint = self.robot_model.get_joint_by_name(name)
            if joint:
                if joint.is_configurable() and name not in self.passive_joints:
                    joints.append(joint)
        return joints

    def get_configurable_joint_names(self, group=None):
        """Get all the :obj:`str` names of configurable joints of a planning group.

        Similar to :meth:`get_configurable_joints` but returning joint names.

        Parameters
        ----------
        group : :obj:`str`, optional
            The name of the planning group. Defaults to the main planning group.

        Returns
        -------
        :obj:`list` of :obj:`str`
        """
        return [joint.name for joint in self.get_configurable_joints(group)]


# XML parsing methods
def _get_groups(root, robot_model):
    groups = {}

    for group in root.findall("group"):
        name = group.attrib["name"]
        groups[name] = dict(
            links=_get_group_link_names(group, root, robot_model),
            joints=_get_group_joint_names(group, root, robot_model),
        )

    return groups


def _get_group_states(root):
    group_states = {}

    for group_state in root.findall("group_state"):
        group_state_name = group_state.attrib["name"]
        group_name = group_state.attrib["group"]
        joint_dict = {joint.attrib["name"]: joint.attrib["value"] for joint in group_state.findall("joint")}
        group_states.setdefault(group_name, {})[group_state_name] = joint_dict

    return group_states


def _get_group_link_names(group, root, robot_model):
    link_names = []
    for link in group.findall("link"):
        name = link.attrib["name"]
        if name not in link_names:
            link_names.append(name)

    for chain in group.findall("chain"):
        for link in robot_model.iter_link_chain(chain.attrib["base_link"], chain.attrib["tip_link"]):
            if link.name not in link_names:
                link_names.append(link.name)

    for joint in group.findall("joint"):
        joint = robot_model.get_joint_by_name(joint.attrib["name"])
        if joint:
            name = joint.parent.link
            if name not in link_names:
                link_names.append(name)

    for subgroup in group.findall("group"):
        if subgroup.attrib["name"] != group.attrib["name"]:
            # find group element at top level
            for top_group_elem in root.findall("group"):
                if top_group_elem.attrib["name"] == subgroup.attrib["name"]:
                    subgroup_link_names = _get_group_link_names(top_group_elem, root, robot_model)
                    for name in subgroup_link_names:
                        if name not in link_names:
                            link_names.append(name)
    return link_names


def _get_group_joint_names(group, root, robot_model):
    joint_names = []
    for link in group.findall("link"):
        link = robot_model.get_link_by_name(link.attrib["name"])
        for joint in link.joints:
            if joint.name not in joint_names:
                joint_names.append(joint.name)

    for chain in group.findall("chain"):
        for joint in robot_model.iter_joint_chain(chain.attrib["base_link"], chain.attrib["tip_link"]):
            if joint.name not in joint_names:
                joint_names.append(joint.name)

    for joint in group.findall("joint"):
        if joint.attrib["name"] not in joint_names:
            joint_names.append(joint.attrib["name"])

    for subgroup in group.findall("group"):
        if subgroup.attrib["name"] != group.attrib["name"]:
            # find group element at top level
            top_group_elem = _get_group_elem_by_name(subgroup.attrib["name"], root)
            subgroup_joint_names = _get_group_joint_names(top_group_elem, root, robot_model)
            for name in subgroup_joint_names:
                if name not in joint_names:
                    joint_names.append(name)
    return joint_names


def _get_group_elem_by_name(group_name, root):
    for group_elem in root.findall("group"):
        if group_elem.attrib["name"] == group_name:
            return group_elem


def _get_passive_joints(root):
    return [joint.attrib["name"] for joint in root.iter("passive_joint")]


def _get_end_effectors(root):
    return [ee.attrib["parent_link"] for ee in root.findall("end_effector")]


def _get_disabled_collisions(root):
    return {tuple([dc.attrib["link1"], dc.attrib["link2"]]) for dc in root.iter("disable_collisions")}

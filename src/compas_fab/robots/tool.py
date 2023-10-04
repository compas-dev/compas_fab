from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json

from compas.data import Data
from compas.robots import Geometry
from compas.robots import ToolModel

from compas_fab.robots.planning_scene import AttachedCollisionMesh
from compas_fab.robots.planning_scene import CollisionMesh


__all__ = ["Tool"]


class Tool(Data):
    """Represents a tool to be attached to the robot's flange.

    Attributes
    ----------
    visual : :class:`compas.datastructures.Mesh`
        The visual mesh of the tool.
    frame_in_tool0_frame : :class:`compas.geometry.Frame`
        The frame of the tool in tool0 frame.
    collision : :class:`compas.datastructures.Mesh`
        The collision mesh representation of the tool.
    name : :obj:`str`
        The name of the `Tool`. Defaults to 'attached_tool'.
    link_name : :obj:`str`
        The name of the `Link` to which the tool is attached.  Defaults to ``None``.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
    >>> tool = Tool(mesh, frame)

    """

    def __init__(self, visual, frame_in_tool0_frame, collision=None, name="attached_tool", link_name=None):
        super(Tool, self).__init__()
        self.tool_model = ToolModel(visual, frame_in_tool0_frame, collision, name, link_name)


    @classmethod
    def from_tool_model(cls, tool_model):
        tool = cls(None, None)
        tool.tool_model = tool_model
        return tool

    @property
    def attached_collision_meshes(self):
        acms = []
        for link in self.tool_model.iter_links():
            for i, item in enumerate(link.collision):
                meshes = Geometry._get_item_meshes(item)
                for mesh in meshes:
                    collision_mesh_name = "{}_{}_collision_{}".format(self.tool_model.name, link.name, i)
                    collision_mesh = CollisionMesh(mesh, collision_mesh_name)
                    attached_collision_mesh = AttachedCollisionMesh(collision_mesh, self.link_name, [self.link_name])
                    acms.append(attached_collision_mesh)
        return acms

    @property
    def name(self):
        return self.tool_model.name

    @property
    def link_name(self):
        return self.tool_model.link_name

    @link_name.setter
    def link_name(self, link_name):
        self.tool_model.link_name = link_name

    @property
    def frame(self):
        return self.tool_model.frame

    @frame.setter
    def frame(self, frame):
        self.tool_model.frame = frame

    @property
    def data(self):
        """Returns the data dictionary that represents the tool.

        Returns
        -------
        :obj:`dict`
            The frame data.

        """
        data = self.tool_model.data
        return data

    @data.setter
    def data(self, data):
        self.tool_model.data = data

    @classmethod
    def from_data(cls, data):
        """Construct a `Tool` from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`Tool`
            The constructed `Tool`.

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> data = {'visual': mesh.data, 'frame': frame.data}
        >>> tool = Tool.from_data(data)
        """
        tool = cls(None, None)
        tool.data = data
        return tool

    @classmethod
    def from_json(cls, filepath):
        """Construct a `Tool` from the data contained in a JSON file.

        Parameters
        ----------
        filepath : str
            Path to the file containing the data.

        Returns
        -------
        :class:`Tool`
            The tool.

        Examples
        --------
        >>> filepath = os.path.join(compas_fab.DATA, "planning_scene", "cone_tool.json")
        >>> tool = Tool.from_json(filepath)
        """
        with open(filepath, "r") as f:
            data = json.load(f)
        return cls.from_data(data)

    def to_json(self, filepath):
        """Serialise the data dictionary representing the tool to JSON and store in a file.

        Parameters
        ----------
        filepath : :obj:`str`
            Path to the file.

        Returns
        -------
        None

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> tool = Tool(mesh, frame)
        >>> filepath = os.path.join(compas_fab.DATA, "planning_scene", "cone_tool.json")
        >>> tool.to_json(filepath)
        """
        with open(filepath, "w") as f:
            json.dump(self.data, f, indent=4, sort_keys=True)

    def update_touch_links(self, touch_links=None):
        if touch_links:
            for acm in self.attached_collision_meshes:
                acm.touch_links = touch_links

    def from_tcf_to_t0cf(self, frames_tcf):
        """Converts a list of frames at the robot's tool tip (tcf frame) to frames at the robot's flange (tool0 frame).

        Parameters
        ----------
        frames_tcf : :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's tool tip (tcf).

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's flange (tool0).

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> tool = Tool(mesh, frame)
        >>> frames_tcf = [Frame((-0.309, -0.046, -0.266), (0.276, 0.926, -0.256), (0.879, -0.136, 0.456))]
        >>> tool.from_tcf_to_t0cf(frames_tcf)
        [Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))]
        """
        return self.tool_model.from_tcf_to_t0cf(frames_tcf)

    def from_t0cf_to_tcf(self, frames_t0cf):
        """Converts frames at the robot's flange (tool0 frame) to frames at the robot's tool tip (tcf frame).

        Parameters
        ----------
        frames_t0cf : :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's flange (tool0).

        Returns
        -------
        :obj:`list` of :class:`compas.geometry.Frame`
            Frames (in WCF) at the robot's tool tip (tcf).

        Examples
        --------
        >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
        >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
        >>> tool = Tool(mesh, frame)
        >>> frames_t0cf = [Frame((-0.363, 0.003, -0.147), (0.388, -0.351, -0.852), (0.276, 0.926, -0.256))]
        >>> tool.from_t0cf_to_tcf(frames_t0cf)
        [Frame(Point(-0.309, -0.046, -0.266), Vector(0.276, 0.926, -0.256), Vector(0.879, -0.136, 0.456))]
        """
        return self.tool_model.from_t0cf_to_tcf(frames_t0cf)

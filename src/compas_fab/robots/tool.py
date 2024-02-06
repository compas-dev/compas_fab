from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.data import Data
from compas_robots import ToolModel
from compas_robots.model import LinkGeometry

from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh

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
    connected_to : :obj:`str`
        The name of the `Link` to which the tool is attached.  Defaults to ``None``.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
    >>> tool = Tool(mesh, frame)

    """

    def __init__(self, visual, frame_in_tool0_frame, collision=None, name="attached_tool", connected_to=None):
        super(Tool, self).__init__()
        self.tool_model = ToolModel(visual, frame_in_tool0_frame, collision, name, connected_to)

    @classmethod
    def from_tool_model(cls, tool_model):
        tool = cls(None, None)
        tool.tool_model = tool_model
        return tool

    @property
    def attached_collision_meshes(self):
        # If the tool model is not set, return an empty list
        if not self.tool_model or len(self.tool_model.links) == 0:
            return []

        acms = []
        for link in self.tool_model.iter_links():
            for i, item in enumerate(link.collision):
                meshes = LinkGeometry._get_item_meshes(item)
                for mesh in meshes:
                    collision_mesh_name = "{}_{}_collision_{}".format(self.tool_model.name, link.name, i)
                    collision_mesh = CollisionMesh(mesh, collision_mesh_name)
                    attached_collision_mesh = AttachedCollisionMesh(
                        collision_mesh, self.connected_to, [self.connected_to]
                    )
                    acms.append(attached_collision_mesh)
        return acms

    @property
    def name(self):
        return self.tool_model.name

    @property
    def connected_to(self):
        return self.tool_model.connected_to

    @connected_to.setter
    def connected_to(self, link_name):
        self.tool_model.connected_to = link_name

    @property
    def frame(self):
        return self.tool_model.frame

    @frame.setter
    def frame(self, frame):
        self.tool_model.frame = frame

    @property
    def __data__(self):
        """Returns the data dictionary that represents the tool.

        Returns
        -------
        :obj:`dict`
            The frame data.

        """
        data = self.tool_model.__data__
        return data

    @classmethod
    def __from_data__(cls, data):
        """Construct a `Tool` from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`Tool`
            The constructed `Tool`.

        """
        tool = cls(None, None)
        tool.tool_model = ToolModel.__from_data__(data)
        return tool

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
        >>> frames_t0cf = tool.from_tcf_to_t0cf(frames_tcf)
        >>> print(frames_t0cf) # doctest: +SKIP
        [Frame(Point(-0.363, 0.003, -0.147), Vector(0.388, -0.351, -0.852), Vector(0.276, 0.926, -0.256))] # doctest: +SKIP
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
        >>> tool.from_t0cf_to_tcf(frames_t0cf)                                                              # doctest: +SKIP
        [Frame(Point(-0.309, -0.046, -0.266), Vector(0.276, 0.926, -0.256), Vector(0.879, -0.136, 0.456))]  # doctest: +SKIP
        """
        return self.tool_model.from_t0cf_to_tcf(frames_t0cf)

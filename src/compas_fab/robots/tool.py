from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json

from compas.geometry import Frame
from compas.datastructures import Mesh

from compas_fab.robots.planning_scene import CollisionMesh


class Tool(object):
    """Represents a tool to be attached to the robot's flange.

    Attributes
    ----------
    visual : :class:`compas.datastructures.Mesh`
        The visual mesh of the tool.
    frame : :class:`compas.geometry.Frame`
        The frame of the tool in tool0 frame.
    collision : :class:`compas.datastructures.Mesh`
        The collision mesh representation of the tool.
    name : str
        The name of the `Tool`. Defaults to 'attached_tool'.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    >>> frame = Frame([0.14, 0, 0], [0, 1, 0], [0, 0, 1])
    >>> tool = Tool(mesh, frame)

    """

    def __init__(self, visual, frame_in_tool0_frame, collision=None,
                 name="attached_tool"):
        self.visual = visual
        self.frame = frame_in_tool0_frame
        self.collision = collision
        self.name = name

    @property
    def collision(self):
        """The collision mesh representation of the tool."""
        return self._collision

    @collision.setter
    def collision(self, collision):
        self._collision = collision or self.visual

    @property
    def collision_mesh(self):
        """Returns the collision mesh of the tool."""
        return CollisionMesh(self.collision, self.name)

    @property
    def data(self):
        """Returns the data dictionary that represents the tool.

        Returns
        -------
        dict
            The frame data.

        """
        return {'visual': self.visual.data,
                'frame': self.frame.data,
                'collision': self.collision.data,
                'name': self.name}

    @data.setter
    def data(self, data):
        self.visual = Mesh.from_data(data['visual'])
        self.frame = Frame.from_data(data['frame'])
        self.collision = Mesh.from_data(data['collision']) if 'collision' in data else None
        self.name = data['name'] if 'name' in data else 'attached_tool'

    @classmethod
    def from_data(cls, data):
        """Construct a `Tool` from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        Tool
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
        Tool
            The tool.

        Examples
        --------
        >>> filepath = os.path.join(compas_fab.DATA, "planning_scene", "cone_tool.json")
        >>> tool = Tool.from_json(filepath)
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls.from_data(data)

    def to_json(self, filepath):
        """Serialise the data dictionary representing the tool to JSON and store in a file.

        Parameters
        ----------
        filepath : str
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
        with open(filepath, 'w') as f:
            json.dump(self.data, f, indent=4, sort_keys=True)

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas import IPY

from compas.data import Data
from compas.datastructures import Mesh
from compas.geometry import Frame
from compas.geometry import Scale

if not IPY:
    from typing import TYPE_CHECKING

    if TYPE_CHECKING:
        from typing import Optional  # noqa: F401
        from compas_fab.backends.interfaces import ClientInterface  # noqa: F401
        from compas_fab.robots import Robot  # noqa: F401
        from compas_fab.robots import Tool  # noqa: F401


__all__ = [
    "AttachedCollisionMesh",
    "CollisionMesh",
    "PlanningScene",
]


class CollisionMesh(Data):
    """Represents a collision mesh.

    Parameters
    ----------
    mesh : :class:`compas.datastructures.Mesh`
        The collision mesh. Ideally it is as coarse as possible.
    id : :obj:`str`
        The id of the mesh, used for identifying it for later operations
    frame : :class:`compas.geometry.Frame`, optional
        The base frame of the mesh.
        Defaults to :meth:`compas.geometry.Frame.worldXY`.
    root_name : :obj:`str`, optional
        The name of the root link the collision mesh will be placed in. Defaults
        to ``'world'``.

    Attributes
    ----------
    mesh : :class:`compas.datastructures.Mesh`
        The collision mesh. Ideally it is as coarse as possible.
    id : :obj:`str`
        The id of the mesh, used for identifying it for later operations
    frame : :class:`compas.geometry.Frame`
        The base frame of the mesh.
        Defaults to :meth:`compas.geometry.Frame.worldXY`.
    root_name : :obj:`str`
        The name of the root link the collision mesh will be placed in. Defaults
        to ``'world'``.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/floor.stl'))
    >>> cm = CollisionMesh(mesh, 'floor')
    >>> cm.frame
    Frame(point=Point(x=0.0, y=0.0, z=0.0), xaxis=Vector(x=1.0, y=0.0, z=0.0), yaxis=Vector(x=0.0, y=1.0, z=0.0))
    """

    def __init__(self, mesh, id, frame=None, root_name=None):
        # type: (Mesh, str, Optional[Frame], Optional[str]) -> None
        super(CollisionMesh, self).__init__()
        self.id = id
        self.mesh = mesh
        self.frame = frame or Frame.worldXY()
        self.root_name = root_name or "world"

    def scale(self, scale_factor):
        # type: (float) -> None
        """Scales the collision mesh uniformly.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor.
        """
        S = Scale.from_factors([scale_factor] * 3)
        self.mesh.transform(S)

    def scaled(self, scale_factor):
        # type: (float) -> None
        """Copies the collision mesh, and scales the copy uniformly.

        Parameters
        ----------
        scale_factor : :obj:`float`
            Scale factor.

        Returns
        -------
        """
        self.mesh = self.mesh.copy()
        self.scale(scale_factor)

    @classmethod
    def __from_data__(cls, data):
        # type: (dict) -> CollisionMesh
        """Construct a collision mesh from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`CollisionMesh`
             An instance of :class:`CollisionMesh`.
        """
        id = data["id"]
        mesh = Mesh.__from_data__(data["mesh"])
        frame = Frame.__from_data__(data["frame"])
        root_name = data["root_name"]

        collision_mesh = cls(mesh, id, frame, root_name)
        return collision_mesh

    @property
    def __data__(self):
        """:obj:`dict` : The data representing the collision mesh."""
        data_obj = {}
        data_obj["id"] = self.id
        data_obj["mesh"] = self.mesh.__data__
        data_obj["frame"] = self.frame.__data__
        data_obj["root_name"] = self.root_name

        return data_obj


class AttachedCollisionMesh(Data):
    """Represents a collision mesh that is attached to a :class:`Robot`'s :class:`compas_robots.model.Link`.

    Parameters
    ----------
    collision_mesh : :class:`compas_fab.robots.CollisionMesh`
        The collision mesh to be attached to the robot model.
    link_name : :obj:`str`
        The name of the :class:`compas_robots.model.Link` the collision mesh will be
        attached to.
    touch_links : :obj:`list` of :obj:`str`, optional
        The list of link names the collision mesh is allowed to touch. Defaults
        to the link it is attached to.
    weight : :obj:`float`, optional
        The weight of the attached object in kg. Defaults to ``1.0``.

    Attributes
    ----------
    collision_mesh : :class:`compas_fab.robots.CollisionMesh`
        The collision mesh we want to attach.
    link_name : :obj:`str`
        The name of the :class:`compas_robots.model.Link` the collision mesh will be
        attached to.
    touch_links : :obj:`list` of :obj:`str`
        The list of link names the collision mesh is allowed to touch. Defaults
        to the link it is attached to.
    weight : :obj:`float`
        The weight of the attached object in kg.

    Examples
    --------
    >>> mesh = Mesh.from_stl(compas_fab.get('planning_scene/cone.stl'))
    >>> cm = CollisionMesh(mesh, 'tip')
    >>> end_effector_link_name = 'tool0'
    >>> touch_links = ['wrist_3_link', 'tool0']
    >>> acm = AttachedCollisionMesh(cm, end_effector_link_name, touch_links)
    """

    def __init__(self, collision_mesh, link_name, touch_links=None, weight=1.0):
        # type: (CollisionMesh, str, Optional[list[str]], Optional[float]) -> None
        super(AttachedCollisionMesh, self).__init__()
        self.collision_mesh = collision_mesh
        if self.collision_mesh:
            self.collision_mesh.root_name = link_name
        self.link_name = link_name
        self.touch_links = touch_links if touch_links else [link_name]
        self.weight = weight

    @classmethod
    def __from_data__(cls, data):
        # type: (dict) -> AttachedCollisionMesh
        """Construct an attached collision mesh from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        :class:`AttachedCollisionMesh`
             An instance of :class:`AttachedCollisionMesh`.
        """
        collision_mesh = CollisionMesh.__from_data__(data["collision_mesh"])
        link_name = data["link_name"]
        touch_links = data["touch_links"]
        weight = data["weight"]

        acm = cls(collision_mesh, link_name, touch_links, weight)
        return acm

    @property
    def __data__(self):
        """:obj:`dict` : The data representing the attached collision mesh."""
        data_obj = {}
        data_obj["collision_mesh"] = self.collision_mesh.__data__
        data_obj["link_name"] = self.link_name
        data_obj["touch_links"] = self.touch_links
        data_obj["weight"] = self.weight

        return data_obj


class PlanningScene(object):
    """Represents the planning scene.

    Parameters
    ----------
    robot : :class:`compas_fab.robots.Robot`
        A reference to the robot in the planning scene.

    Attributes
    ----------
    robot : :class:`compas_fab.robots.Robot`
        A reference to the robot in the planning scene.
    client
        A reference to the robot's backend client.
    """

    def __init__(self, robot):
        # type: (Robot) -> None
        self.robot = robot

    def reset(self):
        # type: () -> None
        """Resets the planning scene, removing all added collision meshes."""
        self.ensure_client()
        self.client.reset_planning_scene()
